#modules/transition_logic/tailsitter_pitch_program.py
import asyncio
import math
from mavsdk.offboard import Attitude, OffboardError


class TailsitterPitchProgram:
    """
    Transition logic for a tailsitter VTOL drone.
    Handles arming, takeoff, throttle and tilt ramping, and failsafes.
    """

    def __init__(self, drone, config, telemetry_handler, logger):
        """
        Initialize the transition logic.

        :param drone: MAVSDK Drone object for controlling the vehicle
        :param config: Configuration dictionary containing operational parameters
        :param telemetry_handler: Telemetry handler for fetching real-time drone telemetry
        :param logger: Logger for detailed output and debugging
        """
        self.drone = drone
        self.config = config
        self.telemetry_handler = telemetry_handler
        self.logger = logger
        self.highest_altitude = 0.0  # Track the highest altitude during transition

    async def execute_transition(self):
        """
        Main execution logic for the VTOL transition process.
        """
        self.logger.info("Starting VTOL transition program.")
        try:
            if self.config.get("enable_takeoff", True):
                await self.arm_and_takeoff()

            await self.start_offboard(retries=3)
            await self.ramp_throttle()
            await self.tilt_transition()
            await self.monitor_and_switch()
        except Exception as e:
            self.logger.error(f"Error during transition: {e}")
            await self.abort_transition()

    async def arm_and_takeoff(self):
        """
        Arm the drone and initiate takeoff.
        """
        self.logger.info("Arming the drone.")
        try:
            await self.drone.action.arm()
            await self.drone.action.set_takeoff_altitude(self.config.get("initial_takeoff_height", 3.0))
            await self.drone.action.takeoff()
            self.logger.info("Takeoff initiated.")
            await asyncio.sleep(5)
        except Exception as e:
            self.logger.error(f"Takeoff failed: {e}")
            await self.abort_transition()

    async def start_offboard(self, retries=3):
        """
        Enter offboard mode with retries for added robustness.
        """
        for attempt in range(retries):
            try:
                await self.drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
                await self.drone.offboard.start()
                self.logger.info("Offboard mode activated.")
                return
            except OffboardError as e:
                self.logger.warning(f"Offboard mode failed on attempt {attempt + 1}/{retries}: {e}")
                await asyncio.sleep(2)
            except Exception as e:
                self.logger.error(f"Unexpected error during offboard start: {e}")
                break

        self.logger.error("Failed to enter offboard mode after retries. Returning to launch.")
        await self.abort_transition()

    async def ramp_throttle(self):
        """
        Gradually ramp throttle over the configured duration.
        """
        max_throttle = self.config.get("max_throttle", 0.8)
        ramp_time = self.config.get("throttle_ramp_time", 5.0)
        steps = int(ramp_time / 0.1)
        throttle_step = max_throttle / steps

        throttle = 0.0
        self.logger.info(f"Ramping throttle over {ramp_time} seconds.")
        for _ in range(steps):
            throttle += throttle_step
            await self.drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, throttle))

            if self.config.get("verbose_mode", False):
                self.logger.debug(f"Throttle: {throttle:.2f}")

            await asyncio.sleep(0.1)

        self.logger.info("Throttle ramp complete.")

    async def tilt_transition(self):
        """
        Gradually tilt the drone to the maximum allowed tilt or beyond if enabled.
        """
        max_tilt = -self.config.get("max_tilt_pitch", 80.0)  # Negative for tilting forward
        forward_transition_time = self.config.get("forward_transition_time", 10.0)
        over_tilt_enabled = self.config.get("over_tilt_enabled", False)
        max_allowed_tilt = -self.config.get("max_allowed_tilt", 120.0)
        altitude_loss_limit = self.config.get("altitude_loss_limit", 10.0)

        steps = int(forward_transition_time / 0.1)
        tilt_step = max_tilt / steps
        tilt = 0.0

        self.logger.info(f"Tilt transition over {forward_transition_time} seconds.")
        for _ in range(steps):
            telemetry = self.telemetry_handler.get_telemetry()
            altitude = -telemetry.get("position_ned", {}).get("down_m", 0.0)
            climb_rate = telemetry.get("fixedwing_metrics", {}).get("climb_rate_m_s", 0.0)

            # Update highest altitude
            self.highest_altitude = max(self.highest_altitude, altitude)

            # Stop tilting if climb rate failsafe is triggered
            if climb_rate < self.config.get("climb_rate_failsafe_threshold", 0.3):
                self.logger.warning("Climb rate below threshold. Stopping tilt progression.")
                break

            # Check altitude loss during over-tilting
            if altitude < (self.highest_altitude - altitude_loss_limit):
                self.logger.error("Excessive altitude loss detected. Aborting transition.")
                await self.abort_transition()
                return

            # Continue tilting
            tilt += tilt_step
            if over_tilt_enabled and tilt < max_allowed_tilt:
                await self.drone.offboard.set_attitude(Attitude(tilt, 0.0, 0.0, self.config.get("max_throttle", 0.8)))

                if self.config.get("verbose_mode", False):
                    self.logger.debug(f"Tilt: {tilt:.2f}, Altitude: {altitude:.2f}")

            await asyncio.sleep(0.1)

        self.logger.info("Tilt transition complete or stopped by criteria.")

    async def monitor_and_switch(self):
        """
        Monitor telemetry and transition to fixed-wing mode when criteria are met.
        """
        start_time = asyncio.get_event_loop().time()
        transition_timeout = self.config.get("transition_timeout", 120.0)

        while True:
            telemetry = self.telemetry_handler.get_telemetry()
            elapsed_time = asyncio.get_event_loop().time() - start_time

            if elapsed_time > transition_timeout:
                self.logger.warning("Transition timeout reached. Aborting transition.")
                await self.abort_transition()
                break

            airspeed = telemetry.get("fixedwing_metrics", {}).get("airspeed_m_s", 0.0)
            if airspeed >= self.config.get("transition_air_speed", 20.0):
                self.logger.info("Airspeed sufficient for transition. Switching to fixed-wing mode.")
                await self.drone.action.transition_to_fixedwing()
                break

            await asyncio.sleep(self.config.get("telemetry_update_interval", 0.1))

    async def abort_transition(self):
        """
        Abort the transition and ensure the drone switches to a safe state.
        """
        self.logger.error("Aborting transition and initiating fail-safe procedures.")
        try:
            if self.config.get("failsafe_multicopter_transition", True):
                await self.drone.action.transition_to_multicopter()
                self.logger.info("Transitioned to multicopter mode for safety.")
        except Exception as e:
            self.logger.warning(f"Error transitioning to multicopter: {e}")

        try:
            await self.drone.offboard.stop()
            self.logger.info("Offboard mode stopped.")
        except Exception as e:
            self.logger.error(f"Error stopping offboard mode: {e}")

        try:
            await self.drone.action.return_to_launch()
            self.logger.info("Return to Launch initiated.")
        except Exception as e:
            self.logger.error(f"Error initiating Return to Launch: {e}")
