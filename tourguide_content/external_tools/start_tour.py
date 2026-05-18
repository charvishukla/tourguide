import logging
import sys
from pathlib import Path
from typing import Any, Dict

# Ensure sibling modules in external_tools/ are importable
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


class StartTour(Tool):

    name = "start_tour"
    description = (
        "Begin the guided tour from the first exhibit. "
        "Call this when the visitor is ready to start the tour. "
        "Queues a greeting emotion and navigates to the first stop."
    )
    parameters_schema = {
        "type": "object",
        "properties": {},
        "required": [],
    }

    async def __call__(self, deps: ToolDependencies, **_kwargs: Any) -> Dict[str, Any]:
        manager = get_tour_state_manager()

        if manager.is_tour_active():
            state = manager.get_state()
            current = manager.get_current_stop()
            return {
                "status": "already_active",
                "phase": state.phase.value,
                "current_stop": current.name if current else None,
                "message": "Tour is already in progress. Use next_exhibit to advance or end_tour to finish.",
            }

        manager.start_tour()

        emotion = _queue_emotion(manager.get_emotion("on_greeting"), deps)

        first_stop = manager.get_stop(0)
        try:
            await get_nav_client().navigate_to_exhibit(first_stop.waypoint_name)
        except NavBridgeError as e:
            logger.error("Navigation bridge error on start_tour: %s", e)
            return {
                "status": "nav_error",
                "error": str(e),
                "message": "Could not reach the navigation bridge. Is the ROSBOT bridge running?",
            }

        manager.begin_navigation_to(0)

        return {
            "status": "started",
            "navigating_to": first_stop.name,
            "total_stops": manager.total_stops(),
            "emotion_queued": emotion,
        }
