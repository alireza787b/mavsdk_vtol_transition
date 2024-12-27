# modules/transition_manager.py

import logging
from .transition_logic.base_transition import BaseTransition
from .transition_logic.tailsitter_pitch_program import TailsitterPitchProgram
# Import other transition classes as needed

class TransitionManager:
    """
    Manages the selection and execution of transition logic.
    """

    TRANSITION_CLASSES = {
        'tailsitter_pitch_program': TailsitterPitchProgram,
        # 'other_transition_type': OtherTransitionClass,
        # Add new transition types here
    }

    def __init__(self, drone, config, telemetry_handler):
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
        self.transition_logic = self._select_transition_logic()

    def _select_transition_logic(self) -> BaseTransition:
        """
        Selects and initializes the appropriate BaseTransition subclass based on configuration.

        :return: Instance of a BaseTransition subclass.
        """
        transition_type = self.config.get('transition_type', 'tailsitter_pitch_program').lower()

        transition_class = self.TRANSITION_CLASSES.get(transition_type)
        if transition_class:
            self.logger.debug(f"Selected {transition_class.__name__} logic.")
            return transition_class(self.drone, self.config, self.telemetry_handler)
        else:
            self.logger.error(f"Unknown transition type: {transition_type}. Defaulting to TailsitterPitchProgram.")
            return TailsitterPitchProgram(self.drone, self.config, self.telemetry_handler)

    async def execute_transition(self):
        """
        Executes the selected transition logic.
        """
        await self.transition_logic.execute_transition()

    async def abort_transition(self):
        """
        Aborts the transition using the selected transition logic.
        """
        await self.transition_logic.abort_transition()
