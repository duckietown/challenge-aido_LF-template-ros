from __future__ import annotations

"""Publish ROS topics for the Duckietown lane-following stack."""

import copy
import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rospy
import yaml
from duckietown_msgs.msg import (
    LEDPattern,
    WheelEncoderStamped,
    WheelsCmdStamped,
)
from sensor_msgs.msg import CameraInfo, CompressedImage


@dataclass(frozen=True)
class _RGB:
    r: float
    g: float
    b: float


_DEFAULT_ENCODER_RESOLUTION = 135
_LOG_DIR = Path("/challenges/challenge-solution-output")
_CALIBRATION_DIR = Path("/data/config/calibrations/camera_intrinsic")


def _camera_frame_id(namespace: str) -> str:
    stripped_namespace = namespace.strip("/")
    return stripped_namespace + "/camera_optical_frame"


def _configure_logging(log_dir: Path) -> None:
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "rosagent-after-init_node.log"
    root_logger = logging.getLogger()
    log_file_name = str(log_file)
    has_log_file_handler = any(
        getattr(handler, "baseFilename", None) == log_file_name
        for handler in root_logger.handlers
    )
    if has_log_file_handler:
        return

    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)
    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(logging.DEBUG)
    root_logger.addHandler(file_handler)
    root_logger.addHandler(stream_handler)


def _vehicle_name() -> str:
    return os.getenv("VEHICLE_NAME", "agent")


class ROSAgent:
    """Mirror Duckiematrix observations onto ROS topics."""

    _cali_file: Path
    _cali_file_folder: Path
    _cam_info_pub: Any
    _cam_pub: Any
    _current_camera_info: CameraInfo
    _frame_id: str
    _ik_action_sub: Any
    _left_encoder_pub: Any
    _led_sub: Any
    _leds: list[_RGB]
    _leds_initialized: bool
    _original_camera_info: CameraInfo
    _right_encoder_pub: Any
    action: np.ndarray
    initialized: bool
    updated: bool
    vehicle: str

    def __init__(self) -> None:
        """Initialize publishers, subscribers, and calibration."""
        self.vehicle = _vehicle_name()

        rospy.init_node(
            "ROSTemplate",
            log_level=rospy.DEBUG,
            disable_rosout=False,
        )
        _configure_logging(_LOG_DIR)

        action_topic = f"/{self.vehicle}/wheels_driver_node/wheels_cmd"
        self._ik_action_sub = rospy.Subscriber(
            action_topic,
            WheelsCmdStamped,
            self._ik_action_cb,
        )
        led_topic = f"/{self.vehicle}/led_emitter_node/led_pattern"
        self._led_sub = rospy.Subscriber(led_topic, LEDPattern, self._led_cb)

        self.action = np.array([0, 0], dtype=float)
        self.updated = True
        self.initialized = False
        self._leds_initialized = False
        self._leds = [_RGB(1, 1, 1)] * 5

        image_topic = f"/{self.vehicle}/camera_node/image/compressed"
        self._cam_pub = rospy.Publisher(
            image_topic,
            CompressedImage,
            queue_size=10,
        )

        camera_info_topic = f"/{self.vehicle}/camera_node/camera_info"
        self._cam_info_pub = rospy.Publisher(
            camera_info_topic,
            CameraInfo,
            queue_size=1,
        )

        left_encoder_topic = f"/{self.vehicle}/left_wheel_encoder_node/tick"
        self._left_encoder_pub = rospy.Publisher(
            left_encoder_topic,
            WheelEncoderStamped,
            queue_size=1,
        )
        right_encoder_topic = f"/{self.vehicle}/right_wheel_encoder_node/tick"
        self._right_encoder_pub = rospy.Publisher(
            right_encoder_topic,
            WheelEncoderStamped,
            queue_size=1,
        )

        self._cali_file_folder = _CALIBRATION_DIR
        namespace = rospy.get_namespace()
        self._frame_id = _camera_frame_id(namespace)
        self._cali_file = self._cali_file_folder / f"{self.vehicle}.yaml"
        if not self._cali_file.is_file():
            warning_message = (
                f"Calibration not found: {self._cali_file}.\n"
                "Using default instead."
            )
            rospy.logwarn(warning_message)
            self._cali_file = self._cali_file_folder / "default.yaml"

        if not self._cali_file.is_file():
            rospy.signal_shutdown("Found no calibration file. Aborting")

        calibration_file = self._cali_file
        self._original_camera_info = self._load_camera_info(calibration_file)
        self._original_camera_info.header.frame_id = self._frame_id
        self._current_camera_info = copy.deepcopy(self._original_camera_info)
        rospy.loginfo(f"Using calibration file: {self._cali_file}")
        rospy.loginfo("Just after init_node.")

    def _ik_action_cb(self, msg: WheelsCmdStamped) -> None:
        """Store the latest inverse-kinematics wheel command."""
        self.initialized = True
        wheel_velocities = [msg.vel_left, msg.vel_right]
        self.action = np.array(wheel_velocities, dtype=float)
        self.updated = True

    def _led_cb(self, msg: LEDPattern) -> None:
        """Store the latest LED pattern emitted by the controller."""
        self._leds_initialized = True
        for index in range(5):
            rgb = msg.rgb_vals[index]
            self._leds[index] = _RGB(rgb.r, rgb.g, rgb.b)
        self.updated = True

    def publish_info(self, timestamp: float) -> None:
        """Publish the current camera info message."""
        stamp = rospy.Time.from_sec(timestamp)
        self._current_camera_info.header.stamp = stamp
        self._cam_info_pub.publish(self._current_camera_info)

    def publish_img(self, obs: bytes, timestamp: float) -> None:
        """Publish a compressed camera image."""
        if not obs:
            return

        stamp = rospy.Time.from_sec(timestamp)
        image_data = bytearray(obs)
        img_message = CompressedImage()
        img_message.header.stamp = stamp
        img_message.format = "jpeg"
        img_message.data = image_data
        self._cam_pub.publish(img_message)

    def publish_encoder_ticks(
        self,
        left_ticks: int,
        right_ticks: int,
        timestamp: float,
        resolution: int = _DEFAULT_ENCODER_RESOLUTION,
    ) -> None:
        """Publish incremental encoder ticks for both wheels."""
        stamp = rospy.Time.from_sec(timestamp)
        encoder_type = WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL

        left_message = WheelEncoderStamped(
            data=left_ticks,
            resolution=resolution,
            type=encoder_type,
        )
        left_message.header.stamp = stamp
        self._left_encoder_pub.publish(left_message)

        right_message = WheelEncoderStamped(
            data=right_ticks,
            resolution=resolution,
            type=encoder_type,
        )
        right_message.header.stamp = stamp
        self._right_encoder_pub.publish(right_message)

    @staticmethod
    def _load_camera_info(filename: Path) -> CameraInfo:
        """Load camera calibration matrices from a YAML file."""
        with filename.open() as stream:
            calibration_data = yaml.safe_load(stream)

        if not isinstance(calibration_data, dict):
            message = f"Unexpected calibration payload in {filename}."
            raise TypeError(message)

        cam_info = CameraInfo()
        cam_info.width = calibration_data["image_width"]
        cam_info.height = calibration_data["image_height"]
        cam_info.K = calibration_data["camera_matrix"]["data"]
        cam_info.D = calibration_data["distortion_coefficients"]["data"]
        cam_info.R = calibration_data["rectification_matrix"]["data"]
        cam_info.P = calibration_data["projection_matrix"]["data"]
        cam_info.distortion_model = calibration_data["distortion_model"]
        return cam_info
