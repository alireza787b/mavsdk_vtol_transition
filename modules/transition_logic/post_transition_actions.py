# modules/transition_logic/post_transition_actions.py

from enum import Enum

class PostTransitionAction(Enum):
    CONTINUE_CURRENT_HEADING = "continue_current_heading"
    HOLD = "hold"
    RETURN_TO_LAUNCH = "return_to_launch"
    START_MISSION_FROM_WAYPOINT = "start_mission_from_waypoint"
