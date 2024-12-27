# modules/transition_logic/base_transition.py

from abc import ABC, abstractmethod
import asyncio
import logging

class BaseTransition(ABC):
    """
    Abstract base class for VTOL transition logic.
    """

    def __init__(self, drone, config, telemetry_handler):
        """
        Initialize the transition logic.

        :param drone: MAVSDK System instance.
        :param config: Configuration dictionary.
        :param telemetry_handler: Instance of TelemetryHandler.
        """
        self.drone = drone
        self.config = config
        self.telemetry_handler = telemetry_handler
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    async def execute_transition(self):
        """
        Execute the transition logic.
        """
        pass

    @abstractmethod
    async def abort_transition(self):
        """
        Abort the transition and initiate fail-safe procedures.
        """
        pass

    # Additional common methods can be defined here if needed
