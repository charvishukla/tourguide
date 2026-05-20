"""Standalone ROS2 HTTP bridge for ROSBOT PRO 3 navigation.

Run this script inside your ROS2 workspace (after sourcing setup.bash):

    python3 ros2_nav_bridge.py \
        --waypoints ../external_content/waypoints.yaml \
        --port 8766

It exposes three HTTP endpoints the conversation app calls over LAN:
    GET  /status      -> current pose + navigation state
    POST /navigate    -> {"exhibit": "lobby"} or {"pose": {x, y, theta}}
    POST /cancel      -> {}

The ROS2 spin loop runs on the main thread; the HTTP server runs on a
daemon thread. Navigation requests from HTTP are handed off to the ROS2
thread via threading.Event to respect action client thread-safety rules.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import threading
from dataclasses import asdict, dataclass
from enum import Enum
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict, Optional

import rclpy
import rclpy.node
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

logger = logging.getLogger(__name__)


class NavigationState(str, Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    FAILED = "failed"


@dataclass
class BridgeStatus:
    state: NavigationState
    current_goal_exhibit: Optional[str]
    pose_x: float
    pose_y: float
    pose_theta: float
    error: Optional[str]


def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert a quaternion to yaw angle in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(theta: float) -> tuple[float, float, float, float]:
    """Convert a yaw angle in radians to a quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


class NavBridgeNode(rclpy.node.Node):
    """ROS2 node wrapping Nav2 NavigateToPose with an HTTP-friendly interface."""

    def __init__(self, waypoints: Dict[str, Dict[str, float]], port: int) -> None:
        super().__init__("nav_bridge")
        self._waypoints = waypoints
        self._lock = threading.Lock()

        self._status = BridgeStatus(
            state=NavigationState.IDLE,
            current_goal_exhibit=None,
            pose_x=0.0,
            pose_y=0.0,
            pose_theta=0.0,
            error=None,
        )

        # Pending navigation request posted by the HTTP thread
        self._pending_nav: Optional[Dict[str, Any]] = None
        self._pending_nav_event = threading.Event()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_callback,
            qos,
        )

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_handle = None

        # Timer used to drain pending navigation requests from the ROS2 thread
        self.create_timer(0.1, self._process_pending_nav)

        logger.info("NavBridgeNode ready on port %d with %d waypoints", port, len(waypoints))

    # ------------------------------------------------------------------ #
    # ROS2 callbacks (run on the spin thread)
    # ------------------------------------------------------------------ #

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        yaw = _quaternion_to_yaw(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        )
        with self._lock:
            self._status.pose_x = p.position.x
            self._status.pose_y = p.position.y
            self._status.pose_theta = yaw

    def _process_pending_nav(self) -> None:
        """Called by the ROS2 timer; drains one pending nav request per tick."""
        if not self._pending_nav_event.is_set():
            return
        self._pending_nav_event.clear()

        with self._lock:
            request = self._pending_nav
            self._pending_nav = None

        if request is None:
            return

        if request.get("cancel"):
            self._do_cancel()
        else:
            self._do_navigate(
                x=request["x"],
                y=request["y"],
                theta=request["theta"],
                exhibit=request.get("exhibit"),
            )

    def _do_navigate(self, x: float, y: float, theta: float, exhibit: Optional[str]) -> None:
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            logger.error("NavigateToPose action server not available")
            with self._lock:
                self._status.state = NavigationState.FAILED
                self._status.error = "Action server unavailable"
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = _yaw_to_quaternion(theta)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        with self._lock:
            self._status.state = NavigationState.NAVIGATING
            self._status.current_goal_exhibit = exhibit
            self._status.error = None

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        logger.info("Navigation goal sent: exhibit=%s (%.2f, %.2f, %.2f)", exhibit, x, y, theta)

    def _do_cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            logger.info("Navigation cancelled")
        with self._lock:
            self._status.state = NavigationState.IDLE
            self._status.current_goal_exhibit = None

    def _goal_response_callback(self, future: Any) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.warning("Navigation goal rejected by Nav2")
            with self._lock:
                self._status.state = NavigationState.FAILED
                self._status.error = "Goal rejected by Nav2"
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: Any) -> None:
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            logger.info("Navigation succeeded")
            with self._lock:
                self._status.state = NavigationState.ARRIVED
                self._status.error = None
        else:
            logger.warning("Navigation ended with status %d", status)
            with self._lock:
                self._status.state = NavigationState.FAILED
                self._status.error = f"Nav2 goal status: {status}"
        self._goal_handle = None

    # ------------------------------------------------------------------ #
    # Public API (called from HTTP thread — thread-safe)
    # ------------------------------------------------------------------ #

    def get_status(self) -> BridgeStatus:
        with self._lock:
            return BridgeStatus(
                state=self._status.state,
                current_goal_exhibit=self._status.current_goal_exhibit,
                pose_x=self._status.pose_x,
                pose_y=self._status.pose_y,
                pose_theta=self._status.pose_theta,
                error=self._status.error,
            )

    def request_navigate_to_waypoint(self, exhibit_name: str) -> tuple[bool, str]:
        wp = self._waypoints.get(exhibit_name)
        if wp is None:
            return False, f"Unknown waypoint '{exhibit_name}'"
        with self._lock:
            if self._status.state == NavigationState.NAVIGATING:
                return False, "Already navigating; cancel first"
            self._pending_nav = {"x": wp["x"], "y": wp["y"], "theta": wp["theta"], "exhibit": exhibit_name}
        self._pending_nav_event.set()
        return True, ""

    def request_navigate_to_pose(self, x: float, y: float, theta: float) -> tuple[bool, str]:
        with self._lock:
            if self._status.state == NavigationState.NAVIGATING:
                return False, "Already navigating; cancel first"
            self._pending_nav = {"x": x, "y": y, "theta": theta, "exhibit": None}
        self._pending_nav_event.set()
        return True, ""

    def request_cancel(self) -> bool:
        with self._lock:
            self._pending_nav = {"cancel": True}
        self._pending_nav_event.set()
        return True


# ------------------------------------------------------------------ #
# HTTP server
# ------------------------------------------------------------------ #

class _Handler(BaseHTTPRequestHandler):
    """Minimal HTTP handler. The HTTPServer subclass holds a ref to the node."""

    def log_message(self, fmt: str, *args: Any) -> None:
        logger.debug("HTTP %s", fmt % args)

    def _send_json(self, code: int, body: Any) -> None:
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self) -> Any:
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length)
        return json.loads(raw) if raw else {}

    def do_GET(self) -> None:
        if self.path != "/status":
            self._send_json(404, {"error": "not found"})
            return
        status = self.server.node.get_status()  # type: ignore[attr-defined]
        self._send_json(200, asdict(status))

    def do_POST(self) -> None:
        try:
            body = self._read_json()
        except Exception:
            self._send_json(400, {"error": "invalid JSON"})
            return

        if self.path == "/navigate":
            self._handle_navigate(body)
        elif self.path == "/cancel":
            self.server.node.request_cancel()  # type: ignore[attr-defined]
            self._send_json(200, {"status": "cancel requested"})
        else:
            self._send_json(404, {"error": "not found"})

    def _handle_navigate(self, body: Any) -> None:
        node = self.server.node  # type: ignore[attr-defined]

        if "exhibit" in body:
            ok, err = node.request_navigate_to_waypoint(body["exhibit"])
            if not ok:
                self._send_json(409, {"error": err})
                return
            self._send_json(200, {"status": "navigating", "exhibit": body["exhibit"]})

        elif "pose" in body:
            pose = body["pose"]
            try:
                x, y, theta = float(pose["x"]), float(pose["y"]), float(pose["theta"])
            except (KeyError, TypeError, ValueError):
                self._send_json(400, {"error": "pose must have x, y, theta"})
                return
            ok, err = node.request_navigate_to_pose(x, y, theta)
            if not ok:
                self._send_json(409, {"error": err})
                return
            self._send_json(200, {"status": "navigating", "pose": {"x": x, "y": y, "theta": theta}})

        else:
            self._send_json(400, {"error": "body must have 'exhibit' or 'pose'"})


class _BridgeHTTPServer(HTTPServer):
    """HTTPServer subclass that carries a reference to the ROS2 node."""

    def __init__(self, node: NavBridgeNode, port: int) -> None:
        super().__init__(("0.0.0.0", port), _Handler)
        self.node = node


# ------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------ #

def load_waypoints(path: str) -> Dict[str, Dict[str, float]]:
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"waypoints file must be a YAML mapping, got {type(data)}")
    for name, wp in data.items():
        for key in ("x", "y", "theta"):
            if key not in wp:
                raise ValueError(f"Waypoint '{name}' missing key '{key}'")
    return data


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    parser = argparse.ArgumentParser(description="ROS2 HTTP navigation bridge for ROSBOT PRO 3")
    parser.add_argument("--port", type=int, default=8766, help="HTTP port (default: 8766)")
    parser.add_argument(
        "--waypoints",
        default="waypoints.yaml",
        help="Path to waypoints.yaml (default: waypoints.yaml)",
    )
    args = parser.parse_args()

    waypoints = load_waypoints(args.waypoints)
    logger.info("Loaded %d waypoints: %s", len(waypoints), list(waypoints.keys()))

    rclpy.init()
    node = NavBridgeNode(waypoints=waypoints, port=args.port)

    http_server = _BridgeHTTPServer(node=node, port=args.port)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    logger.info("HTTP bridge listening on http://0.0.0.0:%d", args.port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        http_server.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        logger.info("Bridge shut down cleanly")


if __name__ == "__main__":
    main()

