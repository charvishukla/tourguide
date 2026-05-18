"""Async HTTP client for the ROS2 navigation bridge.

Not a Tool subclass — imported as a shared helper by tourguide tools:

    from nav_bridge_client import get_nav_client, NavBridgeError, NavStatus
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from typing import Optional

import httpx

logger = logging.getLogger(__name__)

# Override via env var to point at the ROSBOT's IP/port
_BASE_URL = os.getenv("NAV_BRIDGE_URL", "http://localhost:8766")
_TIMEOUT_S = float(os.getenv("NAV_BRIDGE_TIMEOUT", "5.0"))


class NavBridgeError(Exception):
    """Raised when the bridge is unreachable or returns an error response."""


@dataclass
class NavStatus:
    state: str               # "idle" | "navigating" | "arrived" | "failed"
    current_goal_exhibit: Optional[str]
    pose_x: float
    pose_y: float
    pose_theta: float
    error: Optional[str]


class NavBridgeClient:
    """Async HTTP client wrapping the three bridge endpoints."""

    def __init__(self, base_url: str, timeout: float) -> None:
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout

    async def get_status(self) -> NavStatus:
        """GET /status → NavStatus."""
        try:
            async with httpx.AsyncClient(timeout=self._timeout) as client:
                resp = await client.get(f"{self._base_url}/status")
            resp.raise_for_status()
            data = resp.json()
            return NavStatus(
                state=data["state"],
                current_goal_exhibit=data.get("current_goal_exhibit"),
                pose_x=float(data.get("pose_x", 0.0)),
                pose_y=float(data.get("pose_y", 0.0)),
                pose_theta=float(data.get("pose_theta", 0.0)),
                error=data.get("error"),
            )
        except httpx.HTTPError as e:
            raise NavBridgeError(f"Bridge unreachable: {e}") from e
        except Exception as e:
            raise NavBridgeError(f"Unexpected bridge error: {e}") from e

    async def navigate_to_exhibit(self, exhibit_name: str) -> bool:
        """POST /navigate with exhibit name. Returns True on success."""
        try:
            async with httpx.AsyncClient(timeout=self._timeout) as client:
                resp = await client.post(
                    f"{self._base_url}/navigate",
                    json={"exhibit": exhibit_name},
                )
            if resp.status_code == 409:
                raise NavBridgeError(f"Navigation conflict: {resp.json().get('error', '')}")
            resp.raise_for_status()
            return True
        except NavBridgeError:
            raise
        except httpx.HTTPError as e:
            raise NavBridgeError(f"Bridge unreachable: {e}") from e

    async def navigate_to_pose(self, x: float, y: float, theta: float) -> bool:
        """POST /navigate with raw pose. Returns True on success."""
        try:
            async with httpx.AsyncClient(timeout=self._timeout) as client:
                resp = await client.post(
                    f"{self._base_url}/navigate",
                    json={"pose": {"x": x, "y": y, "theta": theta}},
                )
            if resp.status_code == 409:
                raise NavBridgeError(f"Navigation conflict: {resp.json().get('error', '')}")
            resp.raise_for_status()
            return True
        except NavBridgeError:
            raise
        except httpx.HTTPError as e:
            raise NavBridgeError(f"Bridge unreachable: {e}") from e

    async def cancel_navigation(self) -> bool:
        """POST /cancel. Returns True on success."""
        try:
            async with httpx.AsyncClient(timeout=self._timeout) as client:
                resp = await client.post(f"{self._base_url}/cancel", json={})
            resp.raise_for_status()
            return True
        except httpx.HTTPError as e:
            raise NavBridgeError(f"Bridge unreachable: {e}") from e


# ------------------------------------------------------------------ #
# Module-level singleton
# ------------------------------------------------------------------ #

_client: Optional[NavBridgeClient] = None


def get_nav_client() -> NavBridgeClient:
    """Return the process-wide NavBridgeClient, initialising it on first call."""
    global _client
    if _client is None:
        _client = NavBridgeClient(_BASE_URL, _TIMEOUT_S)
        logger.info("NavBridgeClient initialised → %s", _BASE_URL)
    return _client
