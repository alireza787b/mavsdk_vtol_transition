# modules/transition_logic/tailsitter_pitch_program.py

import asyncio
import logging
from mavsdk.offboard import (
    VelocityBodyYawspeed,
    VelocityNedYaw,
    Attitude,
    OffboardError,
)
from .base_transition import BaseTransition

class TailsitterPitchProgram(BaseTransition):
    """
    Transition logic for a tailsitter VTOL drone.
    Handles arming, takeoff, initial climb, throttle ramping, tilt ramping, and failsafes.
    """

    def __init__(self, drone, config: dict, telemetry_handler):
        """
        Initialize the transition logic.

        :param drone: MAVSDK Drone object for controlling the vehicle.
        :param config: Configuration dictionary containing operational parameters.
        :param telemetry_handler: Telemetry handler for fetching real-time drone telemetry.
        """
        super().__init__(drone, config, telemetry_handler)
        self.launch_yaw_angle = 0.0
        self.highest_altitude = 0.0  # Tracks the highest altitude during the transition

    async def execute_transition(self) -> str:
        """
        Main execution logic for the VTOL transition process.

        :return: Status string indicating 'success' or 'failure'.
        """
        self.logger.info("Starting VTOL transition program.")
        try:
            if self.config.get("enable_takeoff", True):
                await self.arm_and_takeoff()

            await self.start_offboard(retries=3)
            await self.initial_climb_phase()
            await self.secondary_climb_phase()
            await self.ramp_throttle_and_tilt()
            status = await self.monitor_and_switch()
            return status

        except Exception as e:
            self.logger.error(f"Error during transition: {e}")
            await self.abort_transition()
            return "failure"

    async def arm_and_takeoff(self) -> None:
        """
        Arm the drone and initiate takeoff.
        """
        self.logger.info("Arming the drone.")
        try:
            await self.drone.action.arm()
            telemetry = self.telemetry_handler.get_telemetry()
            euler_angle = telemetry.get("euler_angle")
            if euler_angle and hasattr(euler_angle, 'yaw_deg'):
                self.launch_yaw_angle = euler_angle.yaw_deg
            else:
                self.launch_yaw_angle = 0.0  # Default yaw angle if not available
                self.logger.warning("Yaw angle telemetry unavailable. Defaulting to 0.0 degrees.")

            await self.drone.action.set_takeoff_altitude(self.config.get("initial_takeoff_height", 3.0))
            await self.drone.action.takeoff()
            self.logger.info("Takeoff initiated.")
            await asyncio.sleep(5)  # Wait for takeoff to stabilize
        except Exception as e:
            self.logger.error(f"Takeoff failed: {e}")
            await self.abort_transition()
            raise

    async def start_offboard(self, retries: int = 3) -> None:
        """
        Enter offboard mode with retries for added robustness.

        :param retries: Number of retry attempts to enter offboard mode.
        """
        for attempt in range(1, retries + 1):
            try:
                # Initialize with zero velocity to start offboard
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                await self.drone.offboard.start()
                self.logger.info("Offboard mode activated.")
                return
            except OffboardError as e:
                self.logger.warning(f"Attempt {attempt}/{retries}: Offboard mode failed - {e}")
                await asyncio.sleep(2)
            except Exception as e:
                self.logger.error(f"Unexpected error during offboard start: {e}")
                break

        self.logger.error("Failed to enter offboard mode after retries. Aborting transition.")
        await self.abort_transition()
        raise RuntimeError("Offboard mode activation failed.")

    async def initial_climb_phase(self) -> None:
        """
        Initial climb phase to reach a preliminary altitude.
        """
        initial_climb_height = self.config.get("initial_climb_height", 5.0)
        initial_climb_rate = self.config.get("initial_climb_rate", 2.0)
        telemetry_interval = self.config.get("telemetry_update_interval", 0.1)

        self.logger.info(f"Starting initial climb to {initial_climb_height} meters at {initial_climb_rate} m/s.")

        while True:
            telemetry = self.telemetry_handler.get_telemetry()
            position_velocity_ned = telemetry.get("position_velocity_ned")
            altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0

            if altitude >= initial_climb_height:
                self.logger.info(f"Reached initial climb height: {altitude:.2f} meters.")
                break

            # Command upward velocity in body frame (positive z is downward)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -initial_climb_rate, 0.0))
            self.logger.debug(f"Initial climb in progress... Current altitude: {altitude:.2f} meters, Target: {initial_climb_height} meters.")
            await asyncio.sleep(telemetry_interval)

    async def secondary_climb_phase(self) -> None:
        """
        Secondary climb phase to transition base altitude.
        """
        transition_base_altitude = self.config.get("transition_base_altitude", 10.0)
        secondary_climb_rate = self.config.get("secondary_climb_rate", 1.0)
        telemetry_interval = self.config.get("telemetry_update_interval", 0.1)

        transition_yaw_angle = self.config.get("transition_yaw_angle", -1)
        if transition_yaw_angle == -1:
            launch_yaw_angle = self.launch_yaw_angle
            self.logger.debug(f"Using launch yaw angle: {launch_yaw_angle} degrees.")
        else:
            launch_yaw_angle = transition_yaw_angle
            self.logger.debug(f"Using configured transition yaw angle: {launch_yaw_angle} degrees.")

        self.logger.info(f"Starting secondary climb to {transition_base_altitude} meters at {secondary_climb_rate} m/s.")

        while True:
            telemetry = self.telemetry_handler.get_telemetry()
            position_velocity_ned = telemetry.get("position_velocity_ned")
            altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0

            if altitude >= transition_base_altitude:
                self.logger.info(f"Reached transition base altitude: {altitude:.2f} meters.")
                break

            # Command upward velocity in NED frame
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -secondary_climb_rate, transition_yaw_angle))
            self.logger.debug(f"Secondary climb in progress... Current altitude: {altitude:.2f} meters, Target: {transition_base_altitude} meters.")
            await asyncio.sleep(telemetry_interval)

    async def ramp_throttle_and_tilt(self) -> None:
        """
        Gradually ramp throttle and tilt over their respective configured durations.
        """
        telemetry_interval = self.config.get("telemetry_update_interval", 0.1)
        telemetry = self.telemetry_handler.get_telemetry()
        fixedwing_metrics = telemetry.get("fixedwing_metrics")
        current_throttle = fixedwing_metrics.throttle_percentage if fixedwing_metrics else 0.7
        self.logger.info(f"Current throttle at the beginning of transition is {current_throttle:.2f}.")

        transition_yaw_angle = self.config.get("transition_yaw_angle", -1)
        if transition_yaw_angle == -1:
            launch_yaw_angle = self.launch_yaw_angle
            self.logger.debug(f"Using launch yaw angle for transition: {launch_yaw_angle} degrees.")
        else:
            launch_yaw_angle = transition_yaw_angle
            self.logger.debug(f"Using configured transition yaw angle: {launch_yaw_angle} degrees.")

        max_throttle = self.config.get("max_throttle", 0.8)
        max_tilt = -self.config.get("max_tilt_pitch", 80.0)  # Negative for downward tilt
        throttle_ramp_time = self.config.get("throttle_ramp_time", 5.0)
        tilt_ramp_time = self.config.get("forward_transition_time", 15.0)

        throttle_steps = int(throttle_ramp_time / telemetry_interval)
        tilt_steps = int(tilt_ramp_time / telemetry_interval)

        throttle_step = (max_throttle - current_throttle) / throttle_steps if throttle_steps > 0 else 0
        tilt_step = max_tilt / tilt_steps if tilt_steps > 0 else 0

        throttle = current_throttle
        tilt = 0.0

        self.logger.info(f"Ramping throttle over {throttle_ramp_time} seconds and tilt over {tilt_ramp_time} seconds.")

        for i in range(max(throttle_steps, tilt_steps)):
            telemetry = self.telemetry_handler.get_telemetry()
            fixedwing_metrics = telemetry.get("fixedwing_metrics")

            if i < throttle_steps:
                throttle += throttle_step
                throttle = min(throttle, max_throttle)  # Ensure throttle does not exceed max
            if i < tilt_steps:
                tilt += tilt_step
                tilt = min(tilt, max_tilt)  # Ensure tilt does not exceed max

            current_tilt_real = telemetry.get("euler_angle").pitch_deg if telemetry.get("euler_angle") else 0.0
            current_airspeed_real = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0

            # Command attitude with updated tilt and throttle
            await self.drone.offboard.set_attitude(Attitude(roll_deg=0.0, pitch_deg=tilt, yaw_deg=launch_yaw_angle, throttle=throttle))
            self.logger.debug(
                f"Throttle: {throttle:.2f}, Tilt Actual/Command: {current_tilt_real:.0f}/{tilt:.0f}, Airspeed: {current_airspeed_real:.0f} m/s"
            )

            await asyncio.sleep(telemetry_interval)

        self.logger.info("Throttle and tilt ramping complete.")

    async def monitor_and_switch(self) -> str:
        """
        Monitor telemetry and transition to fixed-wing mode when criteria are met.

        :return: Status string indicating 'success' or 'failure'.
        """
        start_time = asyncio.get_event_loop().time()
        transition_timeout = self.config.get("transition_timeout", 120.0)
        telemetry_interval = self.config.get("telemetry_update_interval", 0.1)

        while True:
            telemetry = self.telemetry_handler.get_telemetry()
            position_velocity_ned = telemetry.get("position_velocity_ned")
            altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0

            fixedwing_metrics = telemetry.get("fixedwing_metrics")
            airspeed = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0

            elapsed_time = asyncio.get_event_loop().time() - start_time

            if elapsed_time > transition_timeout:
                self.logger.warning("Transition timeout reached. Aborting transition.")
                await self.abort_transition()
                return "failure"

            if airspeed >= self.config.get("transition_air_speed", 20.0):
                self.logger.info("Airspeed sufficient for transition. Switching to fixed-wing mode.")
                status = await self.success_transition()
                return status

            await asyncio.sleep(telemetry_interval)

    async def success_transition(self) -> str:
        """
        Transition to fixed-wing and initiate Hold Flight Mode.

        :return: Status string indicating 'success' or 'failure'.
        """
        self.logger.info("Transition succeeded: performing fixed-wing switch.")

        try:
            # Transition to fixed-wing
            await self.drone.action.transition_to_fixedwing()
            self.logger.info("Transitioned to fixed-wing mode.")

            # Initiate Hold Flight Mode
            await self.drone.action.hold()
            self.logger.info("HOLD mode activated.")

            # Return success status
            return "success"

        except Exception as e:
            self.logger.error(f"Failsafe error during fixed-wing transition: {e}")
            await self.abort_transition()
            return "failure"

    async def abort_transition(self) -> str:
        """
        Abort the transition and ensure the drone switches to a safe state.

        :return: Status string indicating 'failure'.
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

        # Return failure status
        return "failure"
