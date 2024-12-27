# modules/connection_manager.py

import asyncio
import logging
from mavsdk import System
from mavsdk.connection import ConnectionResult

class ConnectionManager:
    """
    Manages the MAVSDK connection to the drone.
    """
    def __init__(self, config: dict):
        self.connection_type = config.get('connection_type', 'udp')
        self.connection_endpoint = config.get('connection_endpoint', 'udp://:14540')
        self.drone = System()
        self.logger = logging.getLogger('ConnectionManager')

    async def connect(self):
        self.logger.info(f"Connecting to drone via {self.connection_type} at {self.connection_endpoint}...")
        await self.drone.connect(system_address=self.connection_endpoint)

        self.logger.info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.logger.info(f"Drone connected with UUID: {state.uuid}")
                break
            await asyncio.sleep(1)
