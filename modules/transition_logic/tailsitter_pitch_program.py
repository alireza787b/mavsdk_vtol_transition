#modules/transition_logic/tailsitter_pitch_program.py
import asyncio
import logging
from mavsdk.offboard import (
    VelocityBodyYawspeed,
    VelocityNedYaw,
    Attitude,
    OffboardError,
)

class TailsitterPitchProgram:
    """
    Transition logic for a tailsitter VTOL drone.
    Handles arming, takeoff, initial climb, throttle ramping, tilt ramping, and failsafes.
    """

    def __init__(self, drone, config, telemetry_handler):
        """
        Initialize the transition logic.

        :param drone: MAVSDK Drone object for controlling the vehicle.
        :param config: Configuration dictionary containing operational parameters.
        :param telemetry_handler: Telemetry handler for fetching real-time drone telemetry.
        """
        self.drone = drone
        self.config = config
        self.telemetry_handler = telemetry_handler
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")  # Hierarchical logger name
        self.highest_altitude = 0.0  # Tracks the highest altitude during the transition

    async def execute_transition(self):
        """
        Main execution logic for the VTOL transition process.
        """
        self.logger.info("Starting VTOL transition program.")
        try:
            if self.config.get("enable_takeoff", True):
                await self.arm_and_takeoff()

            await self.start_offboard(retries=3)
            await self.initial_climb_phase()
            await self.secondary_climb_phase()
            await self.ramp_throttle_and_tilt()
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
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
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

    async def initial_climb_phase(self):
        """
        Initial climb phase to reach a preliminary altitude.
        """
        initial_climb_height = self.config.get("initial_climb_height", 5.0)
        initial_climb_rate = self.config.get("initial_climb_rate", 2.0)
        self.logger.info(f"Starting initial climb to {initial_climb_height} meters at {initial_climb_rate} m/s.")

        while True:
            telemetry = self.telemetry_handler.get_telemetry()
            altitude = -telemetry.get("position_ned", {}).get("down_m", 0.0)

            if altitude >= initial_climb_height:
                self.logger.info(f"Reached initial climb height: {altitude:.2f} meters.")
                break
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -initial_climb_rate, 0.0))
            await asyncio.sleep(self.config.get("telemetry_update_interval", 0.1))

    async def secondary_climb_phase(self):
        """
        Secondary climb phase to transition base altitude.
        """
        transition_base_altitude = self.config.get("transition_base_altitude", 10.0)
        secondary_climb_rate = self.config.get("secondary_climb_rate", 1.0)
        self.logger.info(f"Starting secondary climb to {transition_base_altitude} meters at {secondary_climb_rate} m/s.")

        while True:
            telemetry = self.telemetry_handler.get_telemetry()
            altitude = -telemetry.get("position_ned", {}).get("down_m", 0.0)
            current_yaw = telemetry.get("euler_angle", {}).get("yaw_deg", 0.0)

            if altitude >= transition_base_altitude:
                self.logger.info(f"Reached transition base altitude: {altitude:.2f} meters.")
                break
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -secondary_climb_rate, current_yaw))
            await asyncio.sleep(self.config.get("telemetry_update_interval", 0.1))

    async def ramp_throttle_and_tilt(self):
        """
        Gradually ramp throttle and tilt simultaneously over the configured duration.
        """
        telemetry = self.telemetry_handler.get_telemetry()
        current_throttle = telemetry.get("fixedwing_metrics", {}).get("throttle_percentage", 0.0) / 100.0
        max_throttle = self.config.get("max_throttle", 0.8)
        max_tilt = -self.config.get("max_tilt_pitch", 80.0)
        ramp_time = self.config.get("throttle_ramp_time", 5.0)
        steps = int(ramp_time / 0.1)
        throttle_step = (max_throttle - current_throttle) / steps
        tilt_step = max_tilt / steps

        throttle = current_throttle
        tilt = 0.0

        self.logger.info(f"Ramping throttle and tilt over {ramp_time} seconds.")
        for _ in range(steps):
            throttle += throttle_step
            tilt += tilt_step
            await self.drone.offboard.set_attitude(Attitude(0.0, tilt, 0.0, throttle))

            if self.config.get("verbose_mode", False):
                self.logger.debug(f"Throttle: {throttle:.2f}, Tilt: {tilt:.2f}")

            await asyncio.sleep(0.1)

        self.logger.info("Throttle and tilt ramp complete.")

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
