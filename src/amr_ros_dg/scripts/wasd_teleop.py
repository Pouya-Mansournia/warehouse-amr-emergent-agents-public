#!/usr/bin/env python3
"""
Simple WASD keyboard teleop for amr_ros_dg.

  w = forward       s = backward
  a = turn left      d = turn right
  space = stop (zero velocity)
  q = quit

Publishes geometry_msgs/TwistStamped on /diff_drive_controller/cmd_vel at a steady rate, so
the last commanded velocity keeps being sent until you press another key - this matters
because diff_drive_controller's cmd_vel_timeout (0.5s) will zero the robot's velocity if
nothing is received in time.

NOTE: rclpy spins on a background thread (see main()) precisely so that 10Hz publish rate
is real. sys.stdin.read(1) in get_key() blocks until a key is pressed - if spinning happened
only in the main loop (spin_once() right before each blocking read, as an earlier version of
this script did), the 0.1s timer would only ever fire once per keystroke instead of
continuously, so holding a key relied entirely on the terminal's OS-level key-repeat rate to
keep resending it. That's unreliable (especially over WSL), so commands would drop out
between repeats, cmd_vel_timeout would zero the velocity, and the robot would stutter/lurch
back and forth instead of moving smoothly straight.

NOTE: the config's `use_stamped_vel: false` (config/amr_ros_dg_controllers.yaml) turned out
to be ignored by the diff_drive_controller shipped with ROS 2 Jazzy - confirmed via
`ros2 topic info /diff_drive_controller/cmd_vel -v`, which shows the controller's own
subscription is TwistStamped only, with no plain-Twist subscription at all. Publishing plain
Twist there is a silent no-op (matching type, matching topic name, but zero actual
subscribers see it) - that's why the robot didn't move even though the controller was
active and correctly receiving commands were assumed to be sent.

Usage:
    ros2 run amr_ros_dg wasd_teleop.py
    ros2 run amr_ros_dg wasd_teleop.py --ros-args -r cmd_vel:=/cmd_vel   # if you remap the controller's topic
"""

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

# Teleop operating speeds (defaults, not CAD-derived - see config/amr_ros_dg_controllers.yaml
# header note; kept consistent with that file's linear.x / angular.z max_velocity limits).
LINEAR_SPEED = 0.3   # m/s
ANGULAR_SPEED = 0.8  # rad/s

HELP = """
amr_ros_dg WASD teleop
-----------------------
  w : forward
  s : backward
  a : turn left
  d : turn right
  space : stop
  q : quit
-----------------------
"""

KEY_TO_TWIST = {
    "w": (LINEAR_SPEED, 0.0),
    "s": (-LINEAR_SPEED, 0.0),
    "a": (0.0, ANGULAR_SPEED),
    "d": (0.0, -ANGULAR_SPEED),
    " ": (0.0, 0.0),
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    # Drain any extra buffered bytes and keep only the most recently typed one. Holding a key
    # queues repeated characters from the terminal's own auto-repeat faster than this loop
    # drains them one at a time; without this, a backlog of stale 'w'/'d'/etc keeps getting
    # processed (each re-publishing its own twist) even after the user has already moved on
    # to another key or pressed space - which reads as "space doesn't stop it" or the robot
    # veering off on commands that don't match what's currently being pressed.
    while select.select([sys.stdin], [], [], 0)[0]:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key


class WasdTeleop(Node):
    def __init__(self):
        super().__init__("wasd_teleop")
        self.publisher = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.publish_current_twist)
        self.linear = 0.0
        self.angular = 0.0

    def publish_current_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self.linear
        msg.twist.angular.z = self.angular
        self.publisher.publish(msg)

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0
        self.publish_current_twist()


def main():
    settings = termios.tcgetattr(sys.stdin.fileno())
    rclpy.init()
    node = WasdTeleop()
    print(HELP)

    # Spin on a background thread so the 0.1s publish timer keeps firing at a steady 10Hz
    # regardless of how long the main thread blocks inside get_key() waiting on a keypress -
    # otherwise the timer only ever fires once per keystroke (see module docstring NOTE).
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == "q" or key == "\x03":  # q or Ctrl-C
                break
            if key in KEY_TO_TWIST:
                node.linear, node.angular = KEY_TO_TWIST[key]
                node.publish_current_twist()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        print("\nstopped, exiting.")


if __name__ == "__main__":
    main()
