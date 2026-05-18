import logging
from typing import Any, Dict

from reachy_mini_conversation_app.tools.core_tools import Tool, ToolDependencies

logger = logging.getLogger(__name__)

try:
    from reachy_mini.motion.recorded_move import RecordedMoves
    from reachy_mini_conversation_app.dance_emotion_moves import EmotionQueueMove

    RECORDED_MOVES = RecordedMoves("pollen-robotics/reachy-mini-emotions-library")
    EMOTION_AVAILABLE = True
except ImportError as e:
    logger.warning(f"Emotion library not available: {e}")
    RECORDED_MOVES = None
    EMOTION_AVAILABLE = False

GREETING_EMOTION = "welcoming1"


class TourGuideHello(Tool):

    name = "tourguide_hello_tool"
    description = (
        "Play a greeting emotion when someone says hello or starts a conversation. "
        "Call this at the start of every new interaction."
    )
    parameters_schema = {
        "type": "object",
        "properties": {},
        "required": [],
    }

    async def __call__(self, deps: ToolDependencies, **_kwargs: Any) -> Dict[str, Any]:
        """Play the greeting emotion."""
        if not EMOTION_AVAILABLE:
            logger.warning("Emotion library unavailable; skipping greeting motion")
            return {"status": "skipped", "reason": "emotion library not available"}

        try:
            available = RECORDED_MOVES.list_moves()
            emotion_name = GREETING_EMOTION if GREETING_EMOTION in available else None
            if emotion_name is None:
                logger.warning("Emotion '%s' not found in library; available: %s", GREETING_EMOTION, available)
                return {"status": "error", "reason": f"emotion '{GREETING_EMOTION}' not available"}

            emotion_move = EmotionQueueMove(emotion_name, RECORDED_MOVES)
            deps.movement_manager.queue_move(emotion_move)
            logger.info("Queued greeting emotion: %s", emotion_name)
            return {"status": "queued", "emotion": emotion_name}

        except Exception as e:
            logger.exception("Failed to play greeting emotion")
            return {"status": "error", "reason": str(e)}
