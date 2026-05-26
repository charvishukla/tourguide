"""Standalone ROS2 HTTP server for AprilTag location detection.

Subscribes to /detections (apriltag_msgs/AprilTagDetectionArray),
looks up each detected tag ID in apriltag_locations.yaml, and exposes
GET /apriltag/status for the Reachy conversation app to poll.

Debouncing: a tag must be visible continuously for DEBOUNCE_S seconds
before it counts as a "real" detection (prevents false triggers from
a single noisy frame).

Cooldown: once a tag has been served over HTTP, the same tag ID won't
be reported again for COOLDOWN_S seconds — this stops Reachy from
re-describing the same location every time the conversation app polls.

Usage (after sourcing your ROS2 workspace):

    python3 apriltag_http_server.py \\
        --config path/to/apriltag_locations.yaml \\
        --port 8767

HTTP endpoints:
    GET /apriltag/status  →  detection info as JSON
    GET /health           →  {"status": "ok"}
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import rclpy.node
import yaml

# apriltag_ros publishes apriltag_msgs/msg/AprilTagDetectionArray
from apriltag_msgs.msg import AprilTagDetectionArray

logger = logging.getLogger(__name__)

# How long (seconds) a tag must be continuously visible before it's confirmed.
DEBOUNCE_S: float = 1.0

# How long (seconds) before the same tag can be reported again after being served.
COOLDOWN_S: float = 15.0


# ------------------------------------------------------------------ #
# Data model
# ------------------------------------------------------------------ #

@dataclass
class LocationInfo:
    tag_id: int
    name: str
    description: str
    talking_points: List[str] = field(default_factory=list)


# ------------------------------------------------------------------ #
# Config loader
# ------------------------------------------------------------------ #

def load_locations(path: str) -> Dict[int, LocationInfo]:
    """Load apriltag_locations.yaml and return a {tag_id: LocationInfo} dict."""
    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    if "apriltag_locations" not in raw:
        raise ValueError(
            f"YAML must have a top-level 'apriltag_locations' key, got: {list(raw)}"
        )

    result: Dict[int, LocationInfo] = {}
    for tag_id_raw, entry in raw["apriltag_locations"].items():
        tag_id = int(tag_id_raw)
        result[tag_id] = LocationInfo(
            tag_id=tag_id,
            name=entry["name"],
            description=entry["description"],
            talking_points=entry.get("talking_points", []),
        )
    logger.info("Loaded %d tag location(s): IDs %s", len(result), sorted(result.keys()))
    return result


# ------------------------------------------------------------------ #
# ROS2 Node
# ------------------------------------------------------------------ #

class AprilTagServerNode(rclpy.node.Node):
    """ROS2 node that subscribes to /detections and tracks confirmed tags."""

    def __init__(self, locations: Dict[int, LocationInfo]) -> None:
        super().__init__("apriltag_http_server")
        self._locations = locations
        self._lock = threading.Lock()

        # Debounce state: the tag currently being tracked as a candidate
        self._candidate_id: Optional[int] = None
        self._candidate_family: Optional[str] = None
        self._candidate_first_seen: float = 0.0

        # Confirmed detection waiting to be fetched via HTTP
        # Set to a dict when confirmed, cleared to None after being served.
        self._pending: Optional[Dict[str, Any]] = None

        # Per-tag cooldown timestamps: {tag_id: time_last_served}
        self._cooldowns: Dict[int, float] = {}

        self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self._detections_callback,
            10,
        )

        # Timer to expire stale cooldowns and log heartbeat
        self.create_timer(1.0, self._timer_callback)

        logger.info(
            "AprilTagServerNode ready — debounce=%.1fs, cooldown=%.1fs, locations=%d",
            DEBOUNCE_S,
            COOLDOWN_S,
            len(locations),
        )

    # ------------------------------------------------------------------ #
    # ROS2 callbacks
    # ------------------------------------------------------------------ #

    def _detections_callback(self, msg: AprilTagDetectionArray) -> None:
        now = time.monotonic()

        if not msg.detections:
            # Frame is empty — reset candidate but keep confirmed pending
            with self._lock:
                self._candidate_id = None
                self._candidate_family = None
                self._candidate_first_seen = 0.0
            return

        # Take only the first detection (usually the clearest / closest tag)
        det = msg.detections[0]
        tag_id: int = det.id
        tag_family: str = det.family

        with self._lock:
            # Check cooldown — skip if this tag was recently served
            last_served = self._cooldowns.get(tag_id, 0.0)
            if now - last_served < COOLDOWN_S:
                return

            # Update candidate tracking
            if self._candidate_id != tag_id:
                # New tag appeared — start fresh debounce
                self._candidate_id = tag_id
                self._candidate_family = tag_family
                self._candidate_first_seen = now
                logger.debug("Candidate tag: id=%d family=%s", tag_id, tag_family)
                return

            # Same tag — check debounce window
            if now - self._candidate_first_seen < DEBOUNCE_S:
                return  # Still debouncing

            # Debounce passed — confirm if not already pending
            if self._pending is not None and self._pending.get("tag_id") == tag_id:
                return  # Already confirmed and waiting to be fetched

            location = self._locations.get(tag_id)
            if location is None:
                logger.warning("Tag id=%d has no entry in config — ignoring", tag_id)
                return

            self._pending = {
                "detected": True,
                "tag_id": tag_id,
                "tag_family": tag_family,
                "name": location.name,
                "description": location.description,
                "talking_points": list(location.talking_points),
            }
            logger.info(
                "Tag confirmed: id=%d → '%s' (held for %.1fs)",
                tag_id,
                location.name,
                now - self._candidate_first_seen,
            )

    def _timer_callback(self) -> None:
        """Expire stale cooldowns."""
        now = time.monotonic()
        with self._lock:
            expired = [tid for tid, t in self._cooldowns.items() if now - t >= COOLDOWN_S]
            for tid in expired:
                del self._cooldowns[tid]
                logger.debug("Cooldown expired: tag id=%d", tid)

    # ------------------------------------------------------------------ #
    # Public API — called from HTTP handler thread (must hold lock)
    # ------------------------------------------------------------------ #

    def get_status(self) -> Dict[str, Any]:
        """Return the current detection, consuming it and starting cooldown.

        Returns {"detected": false} when nothing new is available.
        """
        now = time.monotonic()
        with self._lock:
            if self._pending is None:
                return {"detected": False}

            # Consume the pending detection
            result = dict(self._pending)
            tag_id = result["tag_id"]

            # Start cooldown so the same tag isn't immediately re-reported
            self._cooldowns[tag_id] = now
            self._pending = None
            self._candidate_id = None  # reset candidate to require fresh debounce

        logger.info(
            "Served detection: id=%d → '%s' (cooldown %.0fs)",
            tag_id, result["name"], COOLDOWN_S,
        )
        return result


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

    def do_GET(self) -> None:
        if self.path == "/apriltag/status":
            self._send_json(200, self.server.node.get_status())  # type: ignore[attr-defined]
        elif self.path == "/health":
            self._send_json(200, {"status": "ok"})
        else:
            self._send_json(404, {"error": "not found"})


class _AprilTagHTTPServer(HTTPServer):
    def __init__(self, node: AprilTagServerNode, port: int) -> None:
        super().__init__(("0.0.0.0", port), _Handler)
        self.node = node


# ------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------ #

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Default config path: relative to this file inside the workspace
    _default_config = str(
        Path(__file__).resolve().parent.parent.parent
        / "tourguide_bringup"
        / "config"
        / "apriltag_locations.yaml"
    )

    parser = argparse.ArgumentParser(
        description="ROS2 HTTP server for AprilTag location detection"
    )
    parser.add_argument(
        "--config",
        default=_default_config,
        help="Path to apriltag_locations.yaml",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(__import__("os").getenv("APRILTAG_SERVER_PORT", "8767")),
        help="HTTP server port (default: 8767, or $APRILTAG_SERVER_PORT)",
    )
    parser.add_argument(
        "--debounce",
        type=float,
        default=DEBOUNCE_S,
        help=f"Seconds tag must be visible before confirming (default: {DEBOUNCE_S})",
    )
    parser.add_argument(
        "--cooldown",
        type=float,
        default=COOLDOWN_S,
        help=f"Seconds before same tag can be reported again (default: {COOLDOWN_S})",
    )
    args = parser.parse_args()

    # Apply CLI overrides to module-level constants used by the node
    global DEBOUNCE_S, COOLDOWN_S
    DEBOUNCE_S = args.debounce
    COOLDOWN_S = args.cooldown

    locations = load_locations(args.config)

    rclpy.init()
    node = AprilTagServerNode(locations=locations)

    http_server = _AprilTagHTTPServer(node=node, port=args.port)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    logger.info(
        "AprilTag HTTP server listening on http://0.0.0.0:%d/apriltag/status",
        args.port,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        http_server.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        logger.info("AprilTag server shut down cleanly")


if __name__ == "__main__":
    main()
