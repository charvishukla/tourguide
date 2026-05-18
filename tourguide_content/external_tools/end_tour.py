import logging
import sys
from pathlib import Path
from typing import Any, Dict

_TOOLS_DIR = str(Path(__file__).parent)
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from reachy_mini_conversation_app.tools.core_tools import Tool, ToolDependencies

from nav_bridge_client import NavBridgeError, get_nav_client
from tour_state_manager import TourPhase, get_tour_state_manager

logger = logging.getLogger(__name__)

try:
    from reachy_mini.motion.recorded_move import RecordedMoves
    from reachy_mini_conversation_app.dance_emotion_moves import EmotionQueueMove

    RECORDED_MOVES = RecordedMoves("pollen-robotics/reachy-mini-emotions-library")
    EMOTION_AVAILABLE = True
except ImportError as e:
    logger.warning("Emotion library not available: %s", e)
    RECORDED_MOVES = None
    EMOTION_AVAILABLE = False


def _queue_emotion(emotion_name: str | None, deps: ToolDependencies) -> str | None:
    if not emotion_name or not EMOTION_AVAILABLE:
        return None
    available = RECORDED_MOVES.list_moves()
    if emotion_name not in available:
        logger.warning("Emotion '%s' not in library; skipping", emotion_name)
        return None
    deps.movement_manager.queue_move(EmotionQueueMove(emotion_name, RECORDED_MOVES))
    return emotion_name


class EndTour(Tool):

    name = "end_tour"
    description = (
        "Conclude the guided tour. Queues a goodbye emotion and navigates back to the home position. "
        "Call this after the last exhibit has been presented and farewells have been said."
    )
    parameters_schema = {
        "type": "object",
        "properties": {},
        "required": [],
    }

    async def __call__(self, deps: ToolDependencies, **_kwargs: Any) -> Dict[str, Any]:
        manager = get_tour_state_manager()
        state = manager.get_state()

        if state.phase == TourPhase.IDLE:
            return {
                "status": "no_active_tour",
                "message": "No tour is currently active.",
            }

        manager.record_departure()

        emotion = _queue_emotion(manager.get_emotion("on_tour_end"), deps)

        home = manager.get_home_waypoint()
        nav_status = "returning_home"
        try:
            await get_nav_client().navigate_to_exhibit(home)
        except NavBridgeError as e:
            logger.error("Navigation bridge error on end_tour: %s", e)
            nav_status = f"nav_error: {e}"

        visited_names = [manager.get_stop(i).name for i in state.visited_stops]
        dwell = state.dwell_times

        manager.end_tour()

        return {
            "status": "tour_ended",
            "visited_stops": visited_names,
            "dwell_times_seconds": dwell,
            "navigating_to_home": home,
            "nav_status": nav_status,
            "emotion_queued": emotion,
        }
