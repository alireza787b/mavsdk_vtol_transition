# modules/transition_logic/base_transition.py

from abc import ABC, abstractmethod
import asyncio
import logging

class BaseTransition(ABC):
    """
    Abstract base class for VTOL transition logic.
    Provides a standardized interface for executing and aborting transitions.
    """

    def __init__(self, drone, config, telemetry_handler):
        """
        Initialize the transition logic.

        :param drone: MAVSDK System instance.
        :param config: Configuration dictionary.
        :param telemetry_handler: Instance of TelemetryHandler.
        """
        self.drone = drone
        self.fwd_transition_start_time = None
        self.config = config
        self.telemetry_handler = telemetry_handler
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    async def execute_transition(self) -> str:
        """
        Execute the transition logic.

        :return: Status string indicating 'success' or 'failure'.
        """
        pass

    @abstractmethod
    async def abort_transition(self) -> None:
        """
        Abort the transition and initiate fail-safe procedures.
        """
        pass

    # Additional common methods can be defined here if needed
