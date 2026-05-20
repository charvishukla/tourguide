"""Standalone ROS2 HTTP bridge for ROSBOT PRO 3 navigation.

Run this script inside your ROS2 workspace (after sourcing setup.bash):

    python3 ros2_nav_bridge.py \
        --waypoints ../tourguide_content/waypoints.yaml \
        --port 8766 \
        --nav-timeout 120 \
        --arrival-tolerance 0.5

Safeguards implemented:
  - Navigation timeout: if Nav2 takes longer than --nav-timeout seconds,
    state transitions to "timeout" so the LLM can inform the visitor.
  - Arrival pose verification: if Nav2 reports SUCCESS but the robot is
    more than --arrival-tolerance metres from the target, state becomes
    "arrived_uncertain" so the LLM can hedge.
  - GoalStatus classification: ABORTED → no_path, CANCELED → cancelled,
    other → failed, with human-readable error messages.
  - Action server guard: if Nav2 is not running, immediate "failed" with
    clear message rather than hanging.

HTTP endpoints:
    GET  /status    -> BridgeStatus as JSON
    POST /navigate  -> {"exhibit": "name"} or {"pose": {x, y, theta}}
    POST /cancel    -> {}
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import threading
import time
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
    ARRIVED_UNCERTAIN = "arrived_uncertain"  # Nav2 said OK but pose is off
    TIMEOUT = "timeout"                      # Navigation took too long
    NO_PATH = "no_path"                      # Nav2 could not find a path
    CANCELLED = "cancelled"                  # Goal was explicitly cancelled
    FAILED = "failed"                        # Generic Nav2 failure


# Human-readable descriptions sent back to the conversation app
_STATE_DESCRIPTIONS: Dict[str, str] = {
    NavigationState.ARRIVED: "Robot has arrived at the destination.",
    NavigationState.ARRIVED_UNCERTAIN: (
        "Nav2 reported success but the robot may not be precisely at the goal. "
        "Consider verifying position before proceeding."
    ),
    NavigationState.TIMEOUT: (
        "Navigation is taking longer than expected. The robot may be dealing with "
        "an obstacle or running recovery behaviors."
    ),
    NavigationState.NO_PATH: (
        "Nav2 could not find a path to the destination. There may be an unmapped "
        "obstacle blocking the route."
    ),
    NavigationState.CANCELLED: "Navigation was cancelled.",
    NavigationState.FAILED: "Navigation failed for an unknown reason.",
}

_GOAL_STATUS_MAP: Dict[int, NavigationState] = {
    GoalStatus.STATUS_ABORTED: NavigationState.NO_PATH,
    GoalStatus.STATUS_CANCELED: NavigationState.CANCELLED,
}


@dataclass
class BridgeStatus:
    state: NavigationState
    current_goal_exhibit: Optional[str]
    pose_x: float
    pose_y: float
    pose_theta: float
    nav_duration_s: Optional[float]     # seconds since navigation started
    distance_to_goal: Optional[float]   # metres from current pose to target
    error: Optional[str]
    hint: Optional[str]                 # human-readable hint for the LLM


def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(theta: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


def _euclidean(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class NavBridgeNode(rclpy.node.Node):
    """ROS2 node wrapping Nav2 NavigateToPose with safeguards."""

    def __init__(
        self,
        waypoints: Dict[str, Dict[str, float]],
        port: int,
        nav_timeout_s: float,
        arrival_tolerance_m: float,
    ) -> None:
        super().__init__("nav_bridge")
        self._waypoints = waypoints
        self._nav_timeout_s = nav_timeout_s
        self._arrival_tolerance_m = arrival_tolerance_m
        self._lock = threading.Lock()

        self._status = BridgeStatus(
            state=NavigationState.IDLE,
            current_goal_exhibit=None,
            pose_x=0.0,
            pose_y=0.0,
            pose_theta=0.0,
            nav_duration_s=None,
            distance_to_goal=None,
            error=None,
            hint=None,
        )

        # Navigation tracking
        self._nav_start_time: Optional[float] = None
        self._target_x: float = 0.0
        self._target_y: float = 0.0

        # Pending request from HTTP thread
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

        # Main timer: drains pending nav requests + checks timeout
        self.create_timer(0.1, self._timer_callback)

        logger.info(
            "NavBridgeNode ready — timeout=%.0fs, arrival_tolerance=%.2fm, waypoints=%d",
            nav_timeout_s,
            arrival_tolerance_m,
            len(waypoints),
        )

    # ------------------------------------------------------------------ #
    # ROS2 callbacks
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
            if self._status.state == NavigationState.NAVIGATING:
                self._status.distance_to_goal = round(
                    _euclidean(p.position.x, p.position.y, self._target_x, self._target_y), 3
                )

    def _timer_callback(self) -> None:
        """Drain one pending nav request and check for timeout."""
        # Drain pending HTTP request
        if self._pending_nav_event.is_set():
            self._pending_nav_event.clear()
            with self._lock:
                request = self._pending_nav
                self._pending_nav = None
            if request is not None:
                if request.get("cancel"):
                    self._do_cancel()
                else:
                    self._do_navigate(
                        x=request["x"],
                        y=request["y"],
                        theta=request["theta"],
                        exhibit=request.get("exhibit"),
                    )

        # Timeout check
        with self._lock:
            state = self._status.state
            start = self._nav_start_time
        if state == NavigationState.NAVIGATING and start is not None:
            elapsed = time.monotonic() - start
            with self._lock:
                self._status.nav_duration_s = round(elapsed, 1)
            if elapsed > self._nav_timeout_s:
                logger.warning("Navigation timeout after %.0fs", elapsed)
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                with self._lock:
                    self._status.state = NavigationState.TIMEOUT
                    self._status.error = f"Navigation exceeded {self._nav_timeout_s:.0f}s time limit"
                    self._status.hint = _STATE_DESCRIPTIONS[NavigationState.TIMEOUT]
                    self._nav_start_time = None

    def _do_navigate(
        self, x: float, y: float, theta: float, exhibit: Optional[str]
    ) -> None:
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            logger.error("NavigateToPose action server not available")
            with self._lock:
                self._status.state = NavigationState.FAILED
                self._status.error = "Nav2 action server is not running. Check your bringup."
                self._status.hint = (
                    "The navigation stack is not available. "
                    "The robot cannot move until Nav2 is started."
                )
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
            self._status.hint = None
            self._status.distance_to_goal = None
            self._target_x = x
            self._target_y = y
        self._nav_start_time = time.monotonic()

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        logger.info("Goal sent: exhibit=%s (%.2f, %.2f)", exhibit, x, y)

    def _do_cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        with self._lock:
            self._status.state = NavigationState.IDLE
            self._status.current_goal_exhibit = None
            self._status.nav_duration_s = None
            self._status.distance_to_goal = None
        self._nav_start_time = None
        logger.info("Navigation cancelled by request")

    def _goal_response_callback(self, future: Any) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.warning("Navigation goal rejected by Nav2")
            with self._lock:
                self._status.state = NavigationState.FAILED
                self._status.error = "Goal rejected by Nav2"
                self._status.hint = (
                    "Nav2 rejected the navigation goal. "
                    "The robot may already be at the destination or the goal is invalid."
                )
            self._nav_start_time = None
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future: Any) -> None:
        result = future.result()
        status = result.status
        self._goal_handle = None

        with self._lock:
            elapsed = (
                round(time.monotonic() - self._nav_start_time, 1)
                if self._nav_start_time
                else None
            )
            current_x = self._status.pose_x
            current_y = self._status.pose_y

        self._nav_start_time = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            dist = _euclidean(current_x, current_y, self._target_x, self._target_y)
            if dist > self._arrival_tolerance_m:
                # Nav2 said success but robot is too far from target
                logger.warning(
                    "Nav2 SUCCESS but robot is %.2fm from target (tolerance=%.2fm)",
                    dist,
                    self._arrival_tolerance_m,
                )
                with self._lock:
                    self._status.state = NavigationState.ARRIVED_UNCERTAIN
                    self._status.distance_to_goal = round(dist, 3)
                    self._status.nav_duration_s = elapsed
                    self._status.hint = _STATE_DESCRIPTIONS[NavigationState.ARRIVED_UNCERTAIN]
            else:
                logger.info("Navigation succeeded in %.1fs (%.2fm from target)", elapsed or 0, dist)
                with self._lock:
                    self._status.state = NavigationState.ARRIVED
                    self._status.distance_to_goal = round(dist, 3)
                    self._status.nav_duration_s = elapsed
                    self._status.hint = None
        else:
            mapped_state = _GOAL_STATUS_MAP.get(status, NavigationState.FAILED)
            error_msg = f"Nav2 goal status: {status}"
            hint = _STATE_DESCRIPTIONS.get(mapped_state, _STATE_DESCRIPTIONS[NavigationState.FAILED])
            logger.warning("Navigation ended: state=%s status=%d", mapped_state, status)
            with self._lock:
                self._status.state = mapped_state
                self._status.error = error_msg
                self._status.nav_duration_s = elapsed
                self._status.hint = hint

    # ------------------------------------------------------------------ #
    # Public API (called from HTTP thread)
    # ------------------------------------------------------------------ #

    def get_status(self) -> BridgeStatus:
        with self._lock:
            s = self._status
            return BridgeStatus(
                state=s.state,
                current_goal_exhibit=s.current_goal_exhibit,
                pose_x=s.pose_x,
                pose_y=s.pose_y,
                pose_theta=s.pose_theta,
                nav_duration_s=s.nav_duration_s,
                distance_to_goal=s.distance_to_goal,
                error=s.error,
                hint=s.hint,
            )

    def request_navigate_to_waypoint(self, exhibit_name: str) -> tuple[bool, str]:
        wp = self._waypoints.get(exhibit_name)
        if wp is None:
            return False, f"Unknown waypoint '{exhibit_name}'. Available: {list(self._waypoints)}"
        with self._lock:
            if self._status.state == NavigationState.NAVIGATING:
                return False, "Already navigating; POST /cancel first"
            self._pending_nav = {
                "x": wp["x"], "y": wp["y"], "theta": wp["theta"], "exhibit": exhibit_name
            }
        self._pending_nav_event.set()
        return True, ""

    def request_navigate_to_pose(self, x: float, y: float, theta: float) -> tuple[bool, str]:
        with self._lock:
            if self._status.state == NavigationState.NAVIGATING:
                return False, "Already navigating; POST /cancel first"
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
        self._send_json(200, asdict(self.server.node.get_status()))  # type: ignore[attr-defined]

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
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(description="ROS2 HTTP navigation bridge for ROSBOT PRO 3")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--waypoints", default="waypoints.yaml")
    parser.add_argument(
        "--nav-timeout",
        type=float,
        default=120.0,
        help="Seconds before a navigation goal is considered timed out (default: 120)",
    )
    parser.add_argument(
        "--arrival-tolerance",
        type=float,
        default=0.5,
        help="Metres: if robot is further than this from target after Nav2 SUCCESS, "
             "state becomes arrived_uncertain (default: 0.5)",
    )
    args = parser.parse_args()

    waypoints = load_waypoints(args.waypoints)
    logger.info("Loaded %d waypoints: %s", len(waypoints), list(waypoints.keys()))

    rclpy.init()
    node = NavBridgeNode(
        waypoints=waypoints,
        port=args.port,
        nav_timeout_s=args.nav_timeout,
        arrival_tolerance_m=args.arrival_tolerance,
    )

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

