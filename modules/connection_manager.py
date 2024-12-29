# modules/connection_manager.py

import asyncio
import logging
from mavsdk import System, SystemError
from mavsdk.telemetry import ConnectionState

class ConnectionManager:
    """
    Manages the MAVSDK connection to the drone, including connecting and disconnecting.
    """

    def __init__(self, config: dict):
        """
        Initialize the ConnectionManager with configuration parameters.

        :param config: Configuration dictionary containing connection parameters.
        """
        self.connection_type = config.get('connection_type', 'udp')
        self.connection_endpoint = config.get('connection_endpoint', 'udp://:14540')
        self.drone = System(mavsdk_server_address='localhost', port=50051)
        self.logger = logging.getLogger(self.__class__.__name__)
        self.is_connected = False

    async def connect(self) -> bool:
        """
        Establish a connection to the drone.

        :return: True if connected successfully, False otherwise.
        """
        self.logger.info(f"Connecting to drone via {self.connection_type.upper()} at {self.connection_endpoint}...")
        try:
            await self.drone.connect(system_address=self.connection_endpoint)
        except Exception as e:
            self.logger.error(f"Failed to initiate connection: {e}")
            return False

        self.logger.info("Waiting for drone to connect...")
        try:
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("Drone connected successfully!")
                    self.is_connected = True
                    return True
                await asyncio.sleep(1)
        except Exception as e:
            self.logger.error(f"Error while waiting for drone connection: {e}")
            return False

    async def disconnect(self) -> None:
        """
        Disconnect from the drone gracefully.
        """
        if not self.is_connected:
            self.logger.warning("Attempted to disconnect, but no active connection found.")
            return

        
