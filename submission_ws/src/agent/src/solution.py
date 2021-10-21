#!/usr/bin/env python3

import time

import numpy as np

import rospy
from aido_schemas import (
    Context,
    DB20Commands,
    DB20ObservationsWithTimestamp,
    EpisodeStart,
    GetCommands,
    LEDSCommands,
    protocol_agent_DB20_timestamps,
    PWMCommands,
    RGB,
    wrap_direct,
)
from rosagent import ROSAgent


class ROSTemplateAgent:
    last_camera_timestamp: float
    last_odometry_timestamp: float
    agent: ROSAgent

    def __init__(self):

        self.last_camera_timestamp = -1
        self.last_odometry_timestamp = -1

    def init(self, context: Context):
        context.info("init()")
        # Start the ROSAgent, which handles publishing images and subscribing to action
        self.agent = ROSAgent()
        context.info("inited")

    def on_received_seed(self, context: Context, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info(f"Starting episode {data.episode_name}.")
        yaml_payload = getattr(data, "yaml_payload", "{}")
        self.agent.publish_episode_start(data.episode_name, yaml_payload)

    def on_received_observations(self, data: DB20ObservationsWithTimestamp, context: Context):
        camera = data.camera
        odometry = data.odometry
        # context.info(f'received obs camera {camera.timestamp} odometry {odometry.timestamp}')

        if camera.timestamp != self.last_camera_timestamp or True:
            self.agent.publish_img(camera.jpg_data, camera.timestamp)
            self.agent.publish_info(camera.timestamp)
            self.last_camera_timestamp = camera.timestamp

        if odometry.timestamp != self.last_odometry_timestamp or True:
            self.agent.publish_odometry(
                odometry.resolution_rad, odometry.axis_left_rad, odometry.axis_right_rad, odometry.timestamp
            )
            self.last_odometry_timestamp = odometry.timestamp

    def on_received_get_commands(self, context: Context, data: GetCommands):
        # context.info(f'on_received_get_commands')

        if not self.agent.initialized:
            pwm_left, pwm_right = [0, 0]
        else:
            # TODO: let's use a queue here. Performance suffers otherwise.
            # What you should do is: *get the last command*, if available
            # otherwise, wait for one command.
            t0 = time.time()
            while not self.agent.updated:
                dt = time.time() - t0
                if dt > 2.0:
                    context.info(f"agent not ready since {dt:.1f} s")
                    time.sleep(0.5)
                if dt > 60:
                    msg = "I have been waiting for commands from the ROS part" f" since {int(dt)} s"
                    context.error(msg)
                    raise Exception(msg)
                time.sleep(0.02)
            dt = time.time() - t0
            if dt > 2.0:
                context.info(f"obtained agent commands after {dt:.1f} s")
                time.sleep(0.2)

            pwm_left, pwm_right = self.agent.action
            self.agent.updated = False

        grey = RGB(0.5, 0.5, 0.5)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = DB20Commands(pwm_commands, led_commands)

        context.write("commands", commands)

    def finish(self, context):
        context.info("finish()")

        rospy.signal_shutdown("My job here is done.")


if __name__ == "__main__":
    node = ROSTemplateAgent()
    protocol = protocol_agent_DB20_timestamps
    wrap_direct(node=node, protocol=protocol, args=[])
