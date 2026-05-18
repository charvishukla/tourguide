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


class NextExhibit(Tool):

    name = "next_exhibit"
    description = (
        "Move to the next exhibit in the tour sequence. "
        "Call this when you have finished presenting the current exhibit and the visitor is ready to move on. "
        "Do NOT call while nav_state is still 'navigating'."
    )
    parameters_schema = {
        "type": "object",
        "properties": {},
        "required": [],
    }

    async def __call__(self, deps: ToolDependencies, **_kwargs: Any) -> Dict[str, Any]:
        manager = get_tour_state_manager()
        state = manager.get_state()

        if state.phase not in (TourPhase.PRESENTING, TourPhase.QA):
            return {
                "status": "invalid_phase",
                "phase": state.phase.value,
                "message": (
                    "Can only advance from 'presenting' or 'qa' phase. "
                    f"Current phase is '{state.phase.value}'."
                ),
            }

        if not manager.has_next_stop():
            return {
                "status": "no_more_stops",
                "message": "This was the last exhibit. Call end_tour to conclude.",
            }

        manager.record_departure()

        next_stop = manager.get_next_stop()
        emotion = _queue_emotion(manager.get_emotion("on_departure"), deps)

        try:
            await get_nav_client().navigate_to_exhibit(next_stop.waypoint_name)
        except NavBridgeError as e:
            logger.error("Navigation bridge error on next_exhibit: %s", e)
            return {
                "status": "nav_error",
                "error": str(e),
                "message": "Could not reach the navigation bridge. Is the ROSBOT bridge running?",
            }

        manager.begin_navigation_to(next_stop.index)

        remaining_after = manager.total_stops() - (next_stop.index + 1)
        return {
            "status": "navigating",
            "stop": next_stop.name,
            "stop_index": next_stop.index,
            "remaining_after_this": remaining_after,
            "emotion_queued": emotion,
        }
