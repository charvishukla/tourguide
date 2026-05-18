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


class CheckNavigation(Tool):

    name = "check_navigation"
    description = (
        "Poll the ROSBOT navigation status. "
        "Call this on every visitor utterance while the robot is navigating "
        "to know when it has arrived at the destination. "
        "When nav_state is 'arrived', begin presenting the exhibit immediately."
    )
    parameters_schema = {
        "type": "object",
        "properties": {},
        "required": [],
    }

    async def __call__(self, deps: ToolDependencies, **_kwargs: Any) -> Dict[str, Any]:
        manager = get_tour_state_manager()
        state = manager.get_state()

        try:
            nav = await get_nav_client().get_status()
        except NavBridgeError as e:
            logger.error("Navigation bridge unreachable: %s", e)
            return {
                "nav_state": "bridge_unreachable",
                "error": str(e),
                "message": "Cannot reach the navigation bridge. Check that the ROSBOT bridge is running.",
            }

        if nav.state == "arrived" and state.phase == TourPhase.NAVIGATING:
            target = manager.get_target_stop()
            if target is None:
                return {"nav_state": "arrived", "stop": None, "emotion_queued": None}

            manager.arrive_at_stop(target.index)

            arrival_emotion = manager.get_arrival_emotion(target)
            emotion = _queue_emotion(arrival_emotion, deps)

            return {
                "nav_state": "arrived",
                "stop": target.name,
                "description": target.description,
                "talking_points": target.talking_points,
                "stop_number": target.index + 1,
                "total_stops": manager.total_stops(),
                "emotion_queued": emotion,
            }

        if nav.state == "failed":
            return {
                "nav_state": "failed",
                "error": nav.error,
                "message": (
                    "Navigation failed. You may want to let the visitor know and decide "
                    "whether to retry (call next_exhibit or start_tour again) or end the tour."
                ),
            }

        if nav.state == "navigating":
            target = manager.get_target_stop()
            return {
                "nav_state": "navigating",
                "goal": nav.current_goal_exhibit,
                "destination": target.name if target else None,
                "pose": {"x": nav.pose_x, "y": nav.pose_y, "theta": nav.pose_theta},
            }

        # idle — nothing is happening
        return {
            "nav_state": nav.state,
            "tour_phase": state.phase.value,
        }
