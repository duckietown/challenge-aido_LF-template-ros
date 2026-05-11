"""Bridge Duckiematrix world I/O into the local ROS agent."""

from __future__ import annotations

import importlib
import logging
import os
import time
from functools import lru_cache
from threading import Event
from typing import TYPE_CHECKING, Any

from duckietown.sdk.compat import enable_python38_compat

enable_python38_compat()

from duckietown.sdk.middleware.dtps.base import (
    GenericDTPSPublisher,
    GenericDTPSSubscriber,
)
from duckietown.sdk.middleware.shm import ShmWorldInput, ShmWorldOutput
from duckietown.sdk.robots.duckiebot import DB21M
from duckietown_messages.simulation import WorldOutput as WorldOutputMessage

if TYPE_CHECKING:
    JsonObject = dict[str, object]

try:
    from . import rosagent as _rosagent_module
except ImportError:
    _rosagent_module = importlib.import_module("rosagent")

ROSAgent = _rosagent_module.ROSAgent

if not TYPE_CHECKING:
    JsonObject = dict

DEFAULT_VEHICLE_NAME = "map_0/vehicle_0"
SIMULATION_VEHICLE_NAME_ENV = "DUCKIEMATRIX_VEHICLE_NAME"
SDK_ENGINE_HOST = "127.0.0.1"
SDK_ENGINE_PORT = 7501

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def _use_shm_transport() -> bool:
    return bool(os.environ.get("DTSHELL_SHM_PATH", ""))


def _transport_name() -> str:
    return "shm" if _use_shm_transport() else "dtps"


class _DTPSGymWorldInput(GenericDTPSSubscriber):
    def __init__(self, host: str, port: int) -> None:
        super().__init__(host, port, "robot", ("gym", "in"))

    def _unpack(self, message: Any) -> Any:
        return message


class _DTPSGymWorldOutput(GenericDTPSPublisher):
    def __init__(self, host: str, port: int) -> None:
        super().__init__(host, port, "robot", ("gym", "out"))


@lru_cache(maxsize=None)
def _get_vehicle_sdk(vehicle_name: str) -> DB21M:
    return DB21M(vehicle_name, simulated=True, gym_mode=True)


def _normalize_image_bytes(image_data: object) -> bytes | None:
    normalized: bytes | None = None
    if isinstance(image_data, bytes):
        normalized = image_data
    elif isinstance(image_data, bytearray):
        normalized = bytes(image_data)
    elif isinstance(image_data, memoryview):
        normalized = image_data.tobytes()
    elif isinstance(image_data, list) and image_data:
        try:
            normalized = bytes(image_data)
        except ValueError:
            logger.warning("Ignoring invalid list-backed image payload.")
    return normalized or None


def _as_json_object(value: object) -> JsonObject:
    if isinstance(value, dict):
        return value
    return {}


def _as_float(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _as_int(value: object) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            return None
    return None


def _make_world_input(vehicle_name: str) -> Any:
    if _use_shm_transport():
        return ShmWorldInput(
            SDK_ENGINE_HOST,
            SDK_ENGINE_PORT,
            vehicle_name,
            "",
        )
    return _DTPSGymWorldInput(SDK_ENGINE_HOST, SDK_ENGINE_PORT)


def _make_world_output(vehicle_name: str) -> Any:
    if _use_shm_transport():
        return ShmWorldOutput(
            SDK_ENGINE_HOST,
            SDK_ENGINE_PORT,
            vehicle_name,
            "",
        )
    return _DTPSGymWorldOutput(SDK_ENGINE_HOST, SDK_ENGINE_PORT)


def _make_world_output_message(
    vehicle_name: str,
    session_id: int,
    left_pwm: float,
    right_pwm: float,
) -> WorldOutputMessage:
    vehicle_sdk = _get_vehicle_sdk(vehicle_name)
    return WorldOutputMessage(
        session_id=session_id,
        entities={
            vehicle_name: vehicle_sdk.make_world_entity_output(
                left_pwm=float(left_pwm),
                right_pwm=float(right_pwm),
            ),
        },
    )


class _ROSBridge:
    _agent: ROSAgent
    _last_session_id: int | None
    _shutdown_event: Event
    _world_input: Any
    _world_output: Any
    vehicle_name: str

    def __init__(self) -> None:
        """Initialize the bridge and resolve the active vehicle name."""
        self._agent = ROSAgent()
        self._last_session_id = None
        self._shutdown_event = Event()
        self.vehicle_name = self._resolve_vehicle_name()
        self._world_input = _make_world_input(self.vehicle_name)
        self._world_output = _make_world_output(self.vehicle_name)

    def _resolve_vehicle_name(self) -> str:
        preferred_name = os.environ.get(SIMULATION_VEHICLE_NAME_ENV, "").strip()
        if preferred_name:
            return preferred_name
        ros_vehicle_name = os.environ.get("VEHICLE_NAME", "").strip()
        if ros_vehicle_name and "/" in ros_vehicle_name:
            return ros_vehicle_name
        if ros_vehicle_name:
            logger.warning(
                "VEHICLE_NAME=%s is a ROS namespace; using %s for Duckiematrix world I/O. Set %s to override it.",
                ros_vehicle_name,
                DEFAULT_VEHICLE_NAME,
                SIMULATION_VEHICLE_NAME_ENV,
            )
        return DEFAULT_VEHICLE_NAME

    def _publish_observations(self, entity: JsonObject) -> None:
        compressed_image_value = entity.get("compressed_image")
        compressed_image = _as_json_object(compressed_image_value)
        image_header_value = compressed_image.get("header")
        image_header = _as_json_object(image_header_value)
        image_timestamp_value = image_header.get("timestamp")
        image_timestamp = _as_float(image_timestamp_value)
        if image_timestamp is None:
            image_timestamp = time.time()

        raw_image_data = compressed_image.get("data")
        image_data = _normalize_image_bytes(raw_image_data)
        if image_data is not None:
            self._agent.publish_img(image_data, image_timestamp)
            self._agent.publish_info(image_timestamp)

        left_encoder_value = entity.get("left_encoder_ticks")
        left_encoder = _as_json_object(left_encoder_value)
        right_encoder_value = entity.get("right_encoder_ticks")
        right_encoder = _as_json_object(right_encoder_value)
        left_ticks_value = left_encoder.get("data")
        left_ticks = _as_int(left_ticks_value)
        right_ticks_value = right_encoder.get("data")
        right_ticks = _as_int(right_ticks_value)
        if left_ticks is None or right_ticks is None:
            return

        left_header_value = left_encoder.get("header")
        left_header = _as_json_object(left_header_value)
        right_header_value = right_encoder.get("header")
        right_header = _as_json_object(right_header_value)
        left_timestamp_value = left_header.get("timestamp")
        encoder_timestamp = _as_float(left_timestamp_value)
        if encoder_timestamp is None:
            right_timestamp_value = right_header.get("timestamp")
            encoder_timestamp = _as_float(right_timestamp_value)
        if encoder_timestamp is None:
            encoder_timestamp = image_timestamp
        self._agent.publish_encoder_ticks(
            left_ticks,
            right_ticks,
            encoder_timestamp,
        )

    def _publish_actions(self, session_id: int) -> None:
        if self._agent.initialized:
            left_pwm, right_pwm = self._agent.action
        else:
            left_pwm, right_pwm = (0, 0)

        world_output_message = _make_world_output_message(
            self.vehicle_name,
            session_id,
            float(left_pwm),
            float(right_pwm),
        )
        self._world_output.publish(world_output_message)

    def _callback(self, world_input: JsonObject) -> None:
        session_id = world_input.get("session_id")
        if not isinstance(session_id, int):
            message = "WorldInput is missing the required session_id field."
            raise TypeError(message)
        last_session_id = self._last_session_id
        if last_session_id is not None and session_id < last_session_id:
            return
        self._last_session_id = session_id

        entities_value = world_input.get("entities")
        entities = _as_json_object(entities_value)
        entity_value = entities.get(self.vehicle_name)
        entity = _as_json_object(entity_value)
        if not entity:
            logger.warning(
                "WorldInput did not contain data for %s.",
                self.vehicle_name,
            )
            self._publish_actions(session_id)
            return

        self._publish_observations(entity)
        self._publish_actions(session_id)

    def start(self) -> None:
        self._world_input.attach(self._callback)
        self._world_output.start()
        self._world_input.start()

    def wait(self) -> None:
        self._shutdown_event.wait()


def _main() -> None:
    logger.info(
        "ROS template bridge: transport=%s shm=%s engine=%s:%s",
        _transport_name(),
        os.environ.get("DTSHELL_SHM_PATH", ""),
        SDK_ENGINE_HOST,
        SDK_ENGINE_PORT,
    )
    bridge = _ROSBridge()
    logger.info(
        "ROS template bridge: vehicle=%s transport_target=%s",
        bridge.vehicle_name,
        bridge.vehicle_name if _use_shm_transport() else "robot/gym/{in,out}",
    )
    bridge.start()
    bridge.wait()


if __name__ == "__main__":
    _main()
