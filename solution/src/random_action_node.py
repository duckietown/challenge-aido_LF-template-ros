"""Publish placeholder random wheel commands for the ROS template."""

import os

import numpy as np
import rospy
from duckietown_msgs.msg import WheelsCmdStamped

_RNG = np.random.default_rng()


def _continuous_publisher() -> None:
    vehicle = os.getenv("VEHICLE_NAME", "agent")
    topic = f"/{vehicle}/wheels_driver_node/wheels_cmd"
    vel_pub = rospy.Publisher(topic, WheelsCmdStamped, queue_size=1)
    rospy.init_node("random_action_node", anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = WheelsCmdStamped()
        wheel_commands = _RNG.random(2)
        msg.vel_left = wheel_commands[0]
        msg.vel_right = wheel_commands[1]

        vel_pub.publish(msg)
        rate.sleep()


def _main() -> None:
    _continuous_publisher()


if __name__ == "__main__":
    _main()
