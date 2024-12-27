# modules/telemetry_handler.py

import asyncio
import logging
from mavsdk import System
from mavsdk.telemetry import Telemetry

class TelemetryHandler:
    """
    Handles telemetry data retrieval and logging.
    """
    def __init__(self, drone: System, config: dict):
        self.drone = drone
        self.update_interval = config.get('telemetry_update_interval', 1.0)
        self.telemetry_task = None
        self.logger = logging.getLogger('TelemetryHandler')
        self.telemetry_data = {}  # Stores the latest telemetry data

    async def start_telemetry(self):
        self.logger.info("Starting telemetry subscription...")
        try:
            async for telemetry in self.drone.telemetry.subscribe_telemetry():
                self.telemetry_data['altitude'] = telemetry.relative_altitude_m
                self.telemetry_data['airspeed'] = telemetry.airspeed
                self.logger.info(f"Altitude: {telemetry.relative_altitude_m:.2f} m | Airspeed: {telemetry.airspeed:.2f} m/s")
                await asyncio.sleep(self.update_interval)
        except asyncio.CancelledError:
            self.logger.info("Telemetry subscription cancelled.")
        except Exception as e:
            self.logger.error(f"Error in telemetry subscription: {e}")

    def get_telemetry(self):
        """
        Returns the latest telemetry data.
        """
        return self.telemetry_data
