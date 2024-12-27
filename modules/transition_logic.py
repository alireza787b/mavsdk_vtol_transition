# modules/transition_logic.py

import asyncio
import logging
from mavsdk import System
from mavsdk.offboard import OffboardError, AttitudeRate


class VTOLTransition:
    """
    Manages VTOL transition commands and logic.
    """
    def __init__(self, drone: System, config: dict, telemetry_handler):
        self.drone = drone
        self.config = config
        self.telemetry_handler = telemetry_handler
        self.logger = logging.getLogger('VTOLTransition')

    async def arm_and_takeoff(self):
        """
        Arms the drone and initiates takeoff if enabled in the configuration.
        """
        if not self.config.get('enable_takeoff', False):
            self.logger.info("Takeoff disabled by configuration. Skipping arm and takeoff.")
            return

        self.logger.info("Arming the drone...")
        await self.drone.action.arm()

        self.logger.info("Taking off...")
        await self.drone.action.takeoff()

        self.logger.info(f"Waiting to reach safe altitude: {self.config['TRANSITION_SAFE_ALTITUDE']} m")
        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= self.config['TRANSITION_SAFE_ALTITUDE']:
                self.logger.info("Safe altitude reached.")
                break
            await asyncio.sleep(0.5)

    async def start_offboard(self):
        """
        Starts offboard mode for sending commands.
        """
        self.logger.info("Starting offboard mode...")
        try:
            await self.drone.offboard.set_attitude_rate(
                AttitudeRate(
                    roll_deg_s=0.0,
                    pitch_deg_s=0.0,
                    yaw_deg_s=0.0,
                    thrust_value=0.5  # Thrust value between 0.0 and 1.0
                )
            )
            await self.drone.offboard.start()
            self.logger.info("Offboard mode started.")
        except OffboardError as error:
            self.logger.error(f"Failed to start offboard mode: {error._result}")
            await self.drone.action.disarm()
            raise

    async def execute_transition(self):
        """
        Placeholder for transition logic. Currently disabled.
        """
        self.logger.info("Transition logic is currently disabled.")
        try:
            while True:
                # Placeholder: No transition logic implemented
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            self.logger.info("Transition execution cancelled.")
        except Exception as e:
            self.logger.error(f"Error during transition: {e}")
            await self.drone.action.return_to_launch()
