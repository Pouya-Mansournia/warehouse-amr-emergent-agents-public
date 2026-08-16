#!/usr/bin/env python3
"""
Web control dashboard for amr_ros_dg: live camera feeds, lidar view, ultrasonic ring radar,
wheel telemetry, and drive control (keyboard or on-screen D-pad, with adjustable speed
limits) in a single browser page - the same role as wasd_teleop.py, but visual and remote
instead of a terminal-only keyboard loop.

Plain Flask + HTTP polling, no websockets/socketio: the browser polls /api/state (~5Hz)
for telemetry/ultrasonic data, re-fetches /api/camera/<name> on an <img> tag (~5Hz,
motion-JPEG via repeated requests) for the camera/lidar feeds, and POSTs /api/cmd at 10Hz
with the currently-held drive command - the same "keep resending the current command on a
timer so a stale buffer/timeout can't zero it out" approach used in wasd_teleop.py's
background-thread spin fix, just driven from the browser's setInterval instead of a ROS
timer directly. A watchdog zeroes the published velocity if the browser stops polling
(tab closed, network drop) for more than 0.5s, so a lost connection doesn't leave the
robot creeping indefinitely.

Multi-robot: subscribes to EVERY robot in ROBOT_LIST at once (not just one), so the
browser's own robot dropdown can switch which robot's feed/telemetry is shown live,
without restarting this process - every /api/* route below takes a "?robot=robotN" query
param (default: the first robot in the list) rather than relying on a single fixed
ROBOT_NS chosen at process start.

Usage:
    ros2 run amr_ros_dg web_dashboard.py
    # ROBOT_LIST=robot1,robot2,robot3 ros2 run amr_ros_dg web_dashboard.py   (fleet mode)
    # then open http://localhost:8080 in a browser (the WSL host's browser can reach this
    # directly; from Windows, use http://localhost:8080 too - WSL2 forwards it automatically)
"""

import io
import os
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, jsonify, request
from geometry_msgs.msg import TwistStamped
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, LaserScan

CMD_WATCHDOG_TIMEOUT = 0.5  # seconds - matches diff_drive_controller's cmd_vel_timeout
PUBLISH_RATE_HZ = 10.0
ULTRASONIC_COUNT = 12
LIDAR_IMG_SIZE = 480
LIDAR_MAX_RANGE = 8.0  # meters shown at the edge of the lidar view, not the sensor's own max

# ROBOT_LIST: comma-separated robot namespaces to subscribe to, e.g. "robot1,robot2,robot3"
# for a fleet.launch.py run. Empty (default) means single-robot/unnamespaced mode, matching
# gazebo.launch.py's topics ("/camera/d435i", "/scan", ...) - kept as a single "" namespace
# entry internally so the rest of this file doesn't need a separate code path for it.
# ROBOT_NS (old single-namespace env var) still works as a one-robot shorthand for ROBOT_LIST.
_robot_list_env = os.environ.get("ROBOT_LIST", "").strip()
if _robot_list_env:
    ROBOT_LIST = [ns.strip().strip("/") for ns in _robot_list_env.split(",") if ns.strip()]
else:
    ROBOT_LIST = [os.environ.get("ROBOT_NS", "").strip().strip("/")]


def _topic(ns: str, name: str) -> str:
    return f"/{ns}/{name}" if ns else f"/{name}"


def image_msg_to_jpeg(msg: Image) -> bytes:
    """Convert a raw sensor_msgs/Image (rgb8 or bgr8, as published by gz sim's camera
    sensor) to JPEG bytes for serving over HTTP."""
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        arr = arr[:, :, ::-1]
    buf = io.BytesIO()
    PILImage.fromarray(arr, mode="RGB").save(buf, format="JPEG", quality=70)
    return buf.getvalue()


def scan_msg_to_jpeg(msg: LaserScan) -> bytes:
    """Render a LaserScan as a top-down point plot (robot at center, front = up) - the
    same kind of "what does the robot see" view as the camera feeds, just for the lidar
    instead. Plain PIL point-drawing, no matplotlib dependency (this repo already avoids
    one - see the motion-JPEG comments above for the same "keep dependencies minimal"
    pattern)."""
    size = LIDAR_IMG_SIZE
    center = size // 2
    img = PILImage.new("RGB", (size, size), color=(10, 14, 18))
    px = img.load()
    scale = (center - 10) / LIDAR_MAX_RANGE
    angle = msg.angle_min
    for r in msg.ranges:
        if msg.range_min <= r <= msg.range_max:
            # robot's +x (forward) drawn as "up" on screen: screen_x = -r*sin, screen_y = -r*cos
            x = center - r * np.sin(angle) * scale
            y = center - r * np.cos(angle) * scale
            xi, yi = int(x), int(y)
            if 0 <= xi < size and 0 <= yi < size:
                px[xi, yi] = (62, 166, 255)
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    xi2, yi2 = xi + dx, yi + dy
                    if 0 <= xi2 < size and 0 <= yi2 < size:
                        px[xi2, yi2] = (62, 166, 255)
        angle += msg.angle_increment
    # small marker for the robot itself at center
    for dx in range(-4, 5):
        for dy in range(-4, 5):
            xi, yi = center + dx, center + dy
            if 0 <= xi < size and 0 <= yi < size:
                px[xi, yi] = (245, 166, 35)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)
    return buf.getvalue()


class RobotState:
    """Per-robot mutable state - one instance per entry in ROBOT_LIST, so the dashboard can
    hold live data for every robot at once and let the browser pick which one to display."""

    def __init__(self):
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.ultrasonic_ranges = [-1.0] * ULTRASONIC_COUNT
        self.latest_jpeg = {"d435i": None, "floor": None, "lidar": None}
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.actual_linear = 0.0
        self.actual_angular = 0.0
        self.lin_accel = 3.0
        self.ang_accel = 3.0
        self.last_cmd_time = 0.0


class WebDashboardNode(Node):
    def __init__(self):
        super().__init__("web_dashboard")

        self.lock = threading.Lock()
        self.robots = {ns: RobotState() for ns in ROBOT_LIST}
        self.cmd_pubs = {}

        for ns in ROBOT_LIST:
            self.cmd_pubs[ns] = self.create_publisher(
                TwistStamped, _topic(ns, "diff_drive_controller/cmd_vel"), 10
            )
            self.create_subscription(
                JointState, _topic(ns, "joint_states"), self._make_joint_cb(ns), 10
            )
            self.create_subscription(Image, _topic(ns, "camera/d435i"), self._make_image_cb(ns, "d435i"), 1)
            self.create_subscription(Image, _topic(ns, "camera/floor"), self._make_image_cb(ns, "floor"), 1)
            self.create_subscription(LaserScan, _topic(ns, "scan"), self._make_scan_cb(ns), 1)
            for i in range(1, ULTRASONIC_COUNT + 1):
                self.create_subscription(
                    LaserScan, _topic(ns, f"ultrasonic_{i}/scan"), self._make_ultrasonic_cb(ns, i - 1), 1
                )

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_cmd)

    def _make_joint_cb(self, ns: str):
        def cb(msg: JointState):
            with self.lock:
                st = self.robots[ns]
                for name, vel in zip(msg.name, msg.velocity):
                    if name == "left_wheel_joint":
                        st.left_wheel_vel = vel
                    elif name == "right_wheel_joint":
                        st.right_wheel_vel = vel
        return cb

    def _make_image_cb(self, ns: str, key: str):
        def cb(msg: Image):
            try:
                jpeg = image_msg_to_jpeg(msg)
            except Exception:
                return
            with self.lock:
                self.robots[ns].latest_jpeg[key] = jpeg
        return cb

    def _make_scan_cb(self, ns: str):
        def cb(msg: LaserScan):
            try:
                jpeg = scan_msg_to_jpeg(msg)
            except Exception:
                return
            with self.lock:
                self.robots[ns].latest_jpeg["lidar"] = jpeg
        return cb

    def _make_ultrasonic_cb(self, ns: str, index: int):
        def cb(msg: LaserScan):
            r = msg.ranges[0] if msg.ranges else -1.0
            with self.lock:
                self.robots[ns].ultrasonic_ranges[index] = r
        return cb

    def set_cmd(self, ns: str, linear: float, angular: float, lin_accel: float, ang_accel: float):
        with self.lock:
            st = self.robots.get(ns)
            if st is None:
                return
            st.target_linear = linear
            st.target_angular = angular
            # Clamp to sane bounds so a bad/garbage request from the page can't set a zero
            # or negative acceleration (which would freeze the ramp) or an absurdly large one.
            st.lin_accel = min(max(lin_accel, 0.05), 20.0)
            st.ang_accel = min(max(ang_accel, 0.05), 20.0)
            st.last_cmd_time = time.monotonic()

    def stop_now(self, ns: str):
        """Immediate stop, bypassing the acceleration ramp entirely - used by the
        dashboard's Space/STOP button, which should never wait on a deceleration curve."""
        with self.lock:
            st = self.robots.get(ns)
            if st is None:
                return
            st.target_linear = 0.0
            st.target_angular = 0.0
            st.actual_linear = 0.0
            st.actual_angular = 0.0
            st.last_cmd_time = time.monotonic()

    @staticmethod
    def _ramp(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + max_step * (1.0 if delta > 0 else -1.0)

    def _publish_cmd(self):
        dt = 1.0 / PUBLISH_RATE_HZ
        stamp = self.get_clock().now().to_msg()
        for ns, st in self.robots.items():
            with self.lock:
                stale = (time.monotonic() - st.last_cmd_time) > CMD_WATCHDOG_TIMEOUT
                target_linear = 0.0 if stale else st.target_linear
                target_angular = 0.0 if stale else st.target_angular
                st.actual_linear = self._ramp(st.actual_linear, target_linear, st.lin_accel * dt)
                st.actual_angular = self._ramp(st.actual_angular, target_angular, st.ang_accel * dt)
                linear, angular = st.actual_linear, st.actual_angular
            msg = TwistStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular
            self.cmd_pubs[ns].publish(msg)

    def snapshot(self, ns: str) -> dict:
        with self.lock:
            st = self.robots.get(ns)
            if st is None:
                return {}
            return {
                "left_wheel_vel": st.left_wheel_vel,
                "right_wheel_vel": st.right_wheel_vel,
                "ultrasonic": list(st.ultrasonic_ranges),
                "actual_linear": st.actual_linear,
                "actual_angular": st.actual_angular,
            }

    def get_jpeg(self, ns: str, name: str):
        with self.lock:
            st = self.robots.get(ns)
            return st.latest_jpeg.get(name) if st else None


def build_flask_app(ros_node: WebDashboardNode) -> Flask:
    web_dir = os.path.join(get_package_share_directory("amr_ros_dg"), "web")
    with open(os.path.join(web_dir, "dashboard.html"), "r", encoding="utf-8") as f:
        dashboard_html = f.read()

    # 1x1 transparent-ish placeholder JPEG, served until the first real camera frame
    # arrives - keeps the <img> tag from showing a broken-image icon on first load.
    placeholder = io.BytesIO()
    PILImage.new("RGB", (640, 480), color=(15, 20, 26)).save(placeholder, format="JPEG")
    placeholder_jpeg = placeholder.getvalue()

    app = Flask(__name__)

    def _selected_ns() -> str:
        ns = request.args.get("robot", ROBOT_LIST[0])
        return ns if ns in ROBOT_LIST else ROBOT_LIST[0]

    @app.route("/")
    def index():
        return Response(dashboard_html, mimetype="text/html")

    @app.route("/api/robots")
    def api_robots():
        return jsonify({"robots": [ns or "(single-robot)" for ns in ROBOT_LIST], "raw": ROBOT_LIST})

    @app.route("/api/state")
    def api_state():
        return jsonify(ros_node.snapshot(_selected_ns()))

    @app.route("/api/camera/<name>")
    def api_camera(name):
        jpeg = ros_node.get_jpeg(_selected_ns(), name) if name in ("d435i", "floor", "lidar") else None
        return Response(jpeg or placeholder_jpeg, mimetype="image/jpeg")

    @app.route("/api/cmd", methods=["POST"])
    def api_cmd():
        data = request.get_json(force=True, silent=True) or {}
        linear = float(data.get("linear", 0.0))
        angular = float(data.get("angular", 0.0))
        lin_accel = float(data.get("lin_accel", 3.0))
        ang_accel = float(data.get("ang_accel", 3.0))
        ros_node.set_cmd(_selected_ns(), linear, angular, lin_accel, ang_accel)
        return jsonify({"ok": True})

    @app.route("/api/stop", methods=["POST"])
    def api_stop():
        ros_node.stop_now(_selected_ns())
        return jsonify({"ok": True})

    return app


def main():
    rclpy.init()
    node = WebDashboardNode()

    # rclpy spins on a background thread so Flask (handling HTTP requests on its own
    # threads) never blocks sensor callbacks or the cmd_vel publish timer - same reasoning
    # as the background-thread spin fix in wasd_teleop.py.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = build_flask_app(node)
    port = int(os.environ.get("AMR_DASHBOARD_PORT", "8080"))
    node.get_logger().info(
        f"Web dashboard on http://0.0.0.0:{port} (robots: {ROBOT_LIST})"
    )
    try:
        app.run(host="0.0.0.0", port=port, threaded=True)
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
