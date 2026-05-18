"""Tour state manager — shared singleton used by all tourguide tools.

Not a Tool subclass; the tool loader discovers no Tool subclasses here
and skips this file harmlessly. Import via:

    from tour_state_manager import get_tour_state_manager
"""

from __future__ import annotations

import json
import logging
import os
import threading
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

# Paths are relative to this file so they work regardless of cwd
_THIS_DIR = Path(__file__).parent
_CONTENT_DIR = _THIS_DIR.parent  # external_content/

TOUR_CONFIG_PATH = Path(os.getenv("TOUR_CONFIG_PATH", str(_CONTENT_DIR / "tour_config.json")))
STATE_PERSISTENCE_PATH = Path(os.getenv("TOUR_STATE_PATH", str(_CONTENT_DIR / "tour_state.json")))


class TourPhase(str, Enum):
    IDLE = "idle"
    GREETING = "greeting"
    NAVIGATING = "navigating"
    PRESENTING = "presenting"
    QA = "qa"
    DONE = "done"


@dataclass
class TourStop:
    index: int
    name: str
    waypoint_name: str
    description: str
    talking_points: List[str]
    emotion_on_arrival: Optional[str]


@dataclass
class TourState:
    current_stop_index: int = -1      # -1 = before any stop
    target_stop_index: int = -1       # index being navigated to
    phase: TourPhase = TourPhase.IDLE
    visited_stops: List[int] = field(default_factory=list)
    dwell_times: Dict[str, float] = field(default_factory=dict)  # stop_name -> seconds spent
    _arrive_time: float = field(default=0.0, repr=False)


class TourStateManager:
    """Thread-safe singleton managing tour progression and state persistence."""

    def __init__(self, config_path: Path, state_path: Path) -> None:
        self._state_path = state_path
        self._lock = threading.Lock()

        # Load immutable tour config
        with open(config_path, "r") as f:
            raw = json.load(f)

        self._home_waypoint: str = raw["home_waypoint"]
        self._emotions: Dict[str, Optional[str]] = raw.get("emotions", {})
        self._stops: List[TourStop] = [
            TourStop(
                index=s["index"],
                name=s["name"],
                waypoint_name=s["waypoint_name"],
                description=s["description"],
                talking_points=s.get("talking_points", []),
                emotion_on_arrival=s.get("emotion_on_arrival"),
            )
            for s in sorted(raw["stops"], key=lambda x: x["index"])
        ]

        self._state = TourState()
        self._load_state()

        logger.info(
            "TourStateManager loaded: %d stops, phase=%s",
            len(self._stops),
            self._state.phase,
        )

    # ------------------------------------------------------------------ #
    # Config accessors (immutable — no lock needed)
    # ------------------------------------------------------------------ #

    def get_stop(self, index: int) -> TourStop:
        return self._stops[index]

    def get_all_stops(self) -> List[TourStop]:
        return list(self._stops)

    def total_stops(self) -> int:
        return len(self._stops)

    def get_home_waypoint(self) -> str:
        return self._home_waypoint

    def get_emotion(self, event: str) -> Optional[str]:
        """Return the configured emotion name for a global event key, or None."""
        return self._emotions.get(event)

    # ------------------------------------------------------------------ #
    # State mutations (all acquire lock + persist)
    # ------------------------------------------------------------------ #

    def start_tour(self) -> None:
        with self._lock:
            self._state.current_stop_index = -1
            self._state.target_stop_index = -1
            self._state.phase = TourPhase.GREETING
            self._state.visited_stops = []
            self._state.dwell_times = {}
            self._state._arrive_time = 0.0
        self._save_state()
        logger.info("Tour started")

    def begin_navigation_to(self, stop_index: int) -> None:
        with self._lock:
            self._state.target_stop_index = stop_index
            self._state.phase = TourPhase.NAVIGATING
        self._save_state()
        logger.info("Navigating to stop %d (%s)", stop_index, self._stops[stop_index].name)

    def arrive_at_stop(self, stop_index: int) -> None:
        import time
        with self._lock:
            self._state.current_stop_index = stop_index
            self._state.target_stop_index = -1
            self._state.phase = TourPhase.PRESENTING
            if stop_index not in self._state.visited_stops:
                self._state.visited_stops.append(stop_index)
            self._state._arrive_time = time.monotonic()
        self._save_state()
        logger.info("Arrived at stop %d (%s)", stop_index, self._stops[stop_index].name)

    def start_qa(self) -> None:
        with self._lock:
            self._state.phase = TourPhase.QA
        self._save_state()

    def record_departure(self) -> None:
        """Record dwell time for the current stop before departing."""
        import time
        with self._lock:
            idx = self._state.current_stop_index
            arrive = self._state._arrive_time
            if idx >= 0 and arrive > 0:
                dwell = time.monotonic() - arrive
                stop_name = self._stops[idx].name
                self._state.dwell_times[stop_name] = round(dwell, 1)
                logger.info("Dwell time at %s: %.1fs", stop_name, dwell)

    def end_tour(self) -> None:
        with self._lock:
            self._state.phase = TourPhase.DONE
        self._save_state()
        logger.info("Tour ended")

    def reset(self) -> None:
        with self._lock:
            self._state = TourState()
        self._save_state()
        logger.info("Tour state reset to idle")

    # ------------------------------------------------------------------ #
    # State queries (all acquire lock)
    # ------------------------------------------------------------------ #

    def get_state(self) -> TourState:
        with self._lock:
            s = self._state
            return TourState(
                current_stop_index=s.current_stop_index,
                target_stop_index=s.target_stop_index,
                phase=s.phase,
                visited_stops=list(s.visited_stops),
                dwell_times=dict(s.dwell_times),
            )

    def get_current_stop(self) -> Optional[TourStop]:
        with self._lock:
            idx = self._state.current_stop_index
        return self._stops[idx] if 0 <= idx < len(self._stops) else None

    def get_target_stop(self) -> Optional[TourStop]:
        with self._lock:
            idx = self._state.target_stop_index
        return self._stops[idx] if 0 <= idx < len(self._stops) else None

    def get_next_stop(self) -> Optional[TourStop]:
        with self._lock:
            current = self._state.current_stop_index
        next_idx = current + 1
        return self._stops[next_idx] if next_idx < len(self._stops) else None

    def has_next_stop(self) -> bool:
        with self._lock:
            current = self._state.current_stop_index
        return current + 1 < len(self._stops)

    def is_tour_active(self) -> bool:
        with self._lock:
            phase = self._state.phase
        return phase not in (TourPhase.IDLE, TourPhase.DONE)

    def get_arrival_emotion(self, stop: TourStop) -> Optional[str]:
        """Return the emotion to play on arrival: stop-level override or global default."""
        if stop.emotion_on_arrival is not None:
            return stop.emotion_on_arrival
        return self._emotions.get("on_arrival")

    # ------------------------------------------------------------------ #
    # Persistence (atomic write via tmp-rename)
    # ------------------------------------------------------------------ #

    def _serializable_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "current_stop_index": self._state.current_stop_index,
                "target_stop_index": self._state.target_stop_index,
                "phase": self._state.phase.value,
                "visited_stops": list(self._state.visited_stops),
                "dwell_times": dict(self._state.dwell_times),
            }

    def _save_state(self) -> None:
        data = self._serializable_state()
        tmp = self._state_path.with_suffix(".tmp")
        try:
            tmp.write_text(json.dumps(data, indent=2))
            tmp.replace(self._state_path)
        except Exception as e:
            logger.error("Failed to persist tour state: %s", e)

    def _load_state(self) -> None:
        if not self._state_path.exists():
            return
        try:
            data = json.loads(self._state_path.read_text())
            with self._lock:
                self._state.current_stop_index = data.get("current_stop_index", -1)
                self._state.target_stop_index = data.get("target_stop_index", -1)
                self._state.phase = TourPhase(data.get("phase", TourPhase.IDLE.value))
                self._state.visited_stops = data.get("visited_stops", [])
                self._state.dwell_times = data.get("dwell_times", {})
            logger.info("Resumed tour state from disk: phase=%s", self._state.phase)
        except Exception as e:
            logger.warning("Could not load tour state from disk (%s); starting fresh", e)


# ------------------------------------------------------------------ #
# Module-level singleton
# ------------------------------------------------------------------ #

_manager: Optional[TourStateManager] = None
_manager_lock = threading.Lock()


def get_tour_state_manager() -> TourStateManager:
    """Return the process-wide TourStateManager, initialising it on first call."""
    global _manager
    if _manager is None:
        with _manager_lock:
            if _manager is None:
                _manager = TourStateManager(TOUR_CONFIG_PATH, STATE_PERSISTENCE_PATH)
    return _manager
