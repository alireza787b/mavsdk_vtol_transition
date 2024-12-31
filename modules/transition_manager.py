# modules/transition_manager.py

import logging
from typing import Type, Dict
from .transition_logic.base_transition import BaseTransition
from .transition_logic.tailsitter_pitch_program import TailsitterPitchProgram
from .transition_logic.post_transition_actions import PostTransitionAction

# Import other transition classes as needed

class TransitionManager:
    """
    Manages the selection and execution of transition logic.
    Facilitates executing and aborting transitions, and reports their statuses.
    """

    TRANSITION_CLASSES: Dict[str, Type[BaseTransition]] = {
        'tailsitter_pitch_program': TailsitterPitchProgram,
        # 'other_transition_type': OtherTransitionClass,
        # Add new transition types here
    }

    def __init__(self, drone, config: dict, telemetry_handler):
        """
        Initialize the TransitionManager.

        :param drone: MAVSDK System instance.
        :param config: Configuration dictionary.
        :param telemetry_handler: Instance of TelemetryHandler.
        """
        self.drone = drone
        self.config = config
        self.telemetry_handler = telemetry_handler
        self.logger = logging.getLogger(self.__class__.__name__)
        self.transition_logic: BaseTransition = self._select_transition_logic()

    def _select_transition_logic(self) -> BaseTransition:
        """
        Selects and initializes the appropriate BaseTransition subclass based on configuration.
        Defaults to TailsitterPitchProgram if not specified.
        """
        transition_type = self.config.get('transition_type', 'tailsitter_pitch_program').lower()

        transition_class = self.TRANSITION_CLASSES.get(transition_type)
        if transition_class:
            self.logger.debug(f"Selected {transition_class.__name__} logic.")
            return transition_class(self.drone, self.config, self.telemetry_handler)
        else:
            self.logger.error(
                f"Unknown transition type: '{transition_type}'. "
                f"Defaulting to '{TailsitterPitchProgram.__name__}'."
            )
            return TailsitterPitchProgram(self.drone, self.config, self.telemetry_handler)

    async def execute_transition(self) -> str:
        """
        Executes the selected transition logic.

        :return: Status string indicating 'success' or 'failure'.
        """
        self.logger.info(f"Executing transition using '{self.transition_logic.__class__.__name__}'.")
        status = await self.transition_logic.execute_transition()
        self.logger.info(f"Transition execution completed with status: {status}.")
        return status

    async def abort_transition(self) -> None:
        """
        Aborts the transition using the selected transition logic.
        """
        self.logger.info(f"Aborting transition using '{self.transition_logic.__class__.__name__}'.")
        await self.transition_logic.abort_transition()
        self.logger.info("Transition aborted successfully.")
