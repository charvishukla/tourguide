import logging
import sys
from pathlib import Path
from typing import Any, Dict

_TOOLS_DIR = str(Path(__file__).parent)
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from reachy_mini_conversation_app.tools.core_tools import Tool, ToolDependencies

from tour_state_manager import TourPhase, get_tour_state_manager

logger = logging.getLogger(__name__)


class GetTourStatus(Tool):

    name = "get_tour_status"
    description = (
        "Get the current tour status: which exhibit we are at, the phase, talking points, "
        "and how many stops remain. Use this to answer visitor questions like "
        "'where are we?', 'what is here?', or 'how many stops are left?'."
    )
    parameters_schema = {
        "type": "object",
        "properties": {},
        "required": [],
    }

    async def __call__(self, deps: ToolDependencies, **_kwargs: Any) -> Dict[str, Any]:
        manager = get_tour_state_manager()
        state = manager.get_state()

        result: Dict[str, Any] = {
            "phase": state.phase.value,
            "total_stops": manager.total_stops(),
            "visited_count": len(state.visited_stops),
        }

        current = manager.get_current_stop()
        if current is not None:
            result["current_stop"] = {
                "name": current.name,
                "description": current.description,
                "talking_points": current.talking_points,
                "stop_number": current.index + 1,
            }
        else:
            result["current_stop"] = None

        if state.phase == TourPhase.NAVIGATING:
            target = manager.get_target_stop()
            result["navigating_to"] = target.name if target else None

        next_stop = manager.get_next_stop()
        result["next_stop"] = next_stop.name if next_stop else None
        result["stops_remaining"] = manager.total_stops() - len(state.visited_stops)

        if state.dwell_times:
            result["dwell_times_seconds"] = state.dwell_times

        return result
