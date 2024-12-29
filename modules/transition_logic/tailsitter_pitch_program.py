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
    Handles arming, takeoff, initial climb, throttle ramping, tilt ramping, integrated monitoring, and failsafes.
    """

    def __init__(self, drone, config: dict, telemetry_handler):
        """
        Initialize the transition logic.

        :param drone: MAVSDK Drone object for controlling the vehicle.
        :param config: Configuration dictionary containing operational parameters.
        :param telemetry_handler: Telemetry handler for fetching real-time drone telemetry.
        """
        self.drone = drone
        self.config = config
        self.fwd_transition_start_time = None
        self.telemetry_handler = telemetry_handler
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        self.launch_yaw_angle = 0.0  # Stores the yaw angle at launch
        self.highest_altitude = 0.0  # Tracks the highest altitude during the transition

        # Events for inter-task communication
        self.abort_event = asyncio.Event()
        self.transition_event = asyncio.Event()

        # Lock to synchronize offboard command access
        self.command_lock = asyncio.Lock()

    async def execute_transition(self) -> str:
        """
        Main execution logic for the VTOL transition process.

        :return: Status string indicating 'success' or 'failure'.
        """
        self.logger.info("Starting VTOL transition program.")
        try:
            # Phase 1: Arm and Takeoff
            if self.config.get("enable_takeoff", True):
                await self.arm_and_takeoff()

            # Phase 2: Enter Offboard Mode
            await self.start_offboard(retries=3)

            # Phase 3: Initial Climb
            await self.initial_climb_phase()

            # Phase 4: Secondary Climb
            await self.secondary_climb_phase()

            # Phase 5: Ramping and Monitoring
            # Start ramping and monitoring as separate tasks
            ramping_task = asyncio.create_task(self.ramp_throttle_and_tilt())
            monitoring_task = asyncio.create_task(self.monitor_and_switch())

            # Wait for either task to complete
            done, pending = await asyncio.wait(
                [ramping_task, monitoring_task],
                return_when=asyncio.FIRST_COMPLETED
            )

            # If monitoring task completed first, check the result
            for task in done:
                result = task.result()
                if result == "success":
                    self.logger.info("Transition executed successfully.")
                    # Cancel the ramping task if it's still running
                    for pending_task in pending:
                        pending_task.cancel()
                    return "success"
                elif result == "failure":
                    self.logger.warning("Transition failed during monitoring.")
                    # Cancel the ramping task if it's still running
                    for pending_task in pending:
                        pending_task.cancel()
                    return "failure"

            # If ramping completes without triggering monitoring, wait for monitoring to finish
            await asyncio.gather(monitoring_task, return_exceptions=True)
            self.logger.info("Transition execution completed.")
            return "success"

        except asyncio.CancelledError:
            self.logger.warning("Transition execution was cancelled.")
            await self.abort_transition()
            return "failure"
        except Exception as e:
            self.logger.error(f"Unexpected error during transition: {e}")
            await self.abort_transition()
            return "failure"

    async def arm_and_takeoff(self) -> None:
        """
        Arm the drone and initiate takeoff.
        """
        self.logger.info("Arming the drone.")
        try:
            async with self.command_lock:
                await self.drone.action.arm()
                await self.drone.action.set_takeoff_altitude(
                    self.config.get("initial_takeoff_height", 3.0)
                )
                await self.drone.action.takeoff()
            self.logger.info("Takeoff initiated.")
            await asyncio.sleep(5)  # Wait for takeoff to stabilize
        except Exception as e:
            self.logger.error(f"Takeoff failed: {e}")
            await self.abort_transition()
            raise  # Re-raise to be caught in execute_transition

    async def start_offboard(self, retries: int = 3) -> None:
        """
        Enter offboard mode with retries for added robustness.

        :param retries: Number of retry attempts to enter offboard mode.
        """
        for attempt in range(1, retries + 1):
            try:
                async with self.command_lock:
                    # Initialize with zero velocity to start offboard
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                    )
                    await self.drone.offboard.start()
                self.logger.info("Offboard mode activated.")
                return
            except OffboardError as e:
                self.logger.warning(
                    f"Attempt {attempt}/{retries}: Offboard mode failed - {e}"
                )
                await asyncio.sleep(2)
            except Exception as e:
                self.logger.error(
                    f"Unexpected error during offboard start: {e}"
                )
                break

        self.logger.error(
            "Failed to enter offboard mode after retries. Aborting transition."
        )
        await self.abort_transition()
        raise RuntimeError("Offboard mode activation failed.")

    async def initial_climb_phase(self) -> None:
        """
        Initial climb phase to reach a preliminary altitude.
        """
        initial_climb_height = self.config.get("initial_climb_height", 5.0)
        initial_climb_rate = self.config.get("initial_climb_rate", 2.0)
        telemetry_interval = self.config.get("telemetry_update_interval", 0.1)

        self.logger.info(
            f"Starting initial climb to {initial_climb_height} meters at {initial_climb_rate} m/s."
        )

        try:
            while True:
                telemetry = self.telemetry_handler.get_telemetry()
                position_velocity_ned = telemetry.get("position_velocity_ned")
                altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0

                if altitude >= initial_climb_height:
                    self.logger.info(f"Reached initial climb height: {altitude:.2f} meters.")
                    break

                # Command upward velocity in body frame (positive z is downward)
                async with self.command_lock:
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, -initial_climb_rate, 0.0)
                    )
                self.logger.info(
                    f"Initial climb in progress... Current altitude: {altitude:.2f} meters, Target: {initial_climb_height} meters."
                )
                await asyncio.sleep(telemetry_interval)
        except asyncio.CancelledError:
            self.logger.warning("Initial climb phase was cancelled.")
            raise
        except Exception as e:
            self.logger.error(f"Error during initial climb phase: {e}")
            await self.abort_transition()
            raise

    async def secondary_climb_phase(self) -> None:
        """
        Secondary climb phase to reach transition base altitude.
        """
        transition_base_altitude = self.config.get("transition_base_altitude", 10.0)
        secondary_climb_rate = self.config.get("secondary_climb_rate", 1.0)
        transition_yaw_angle = self.config.get("transition_yaw_angle", 0.0)
        telemetry_interval = self.config.get("telemetry_update_interval", 0.1)

        self.logger.info(
            f"Starting secondary climb to {transition_base_altitude} meters at {secondary_climb_rate} m/s."
        )

        try:
            while True:
                telemetry = self.telemetry_handler.get_telemetry()
                position_velocity_ned = telemetry.get("position_velocity_ned")
                altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0

                if altitude >= transition_base_altitude:
                    self.logger.info(f"Reached transition base altitude: {altitude:.2f} meters.")
                    break

                # Command upward velocity in NED frame
                async with self.command_lock:
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, -secondary_climb_rate, transition_yaw_angle)
                    )
                self.logger.info(
                    f"Secondary climb in progress... Current altitude: {altitude:.2f} meters, Target: {transition_base_altitude} meters."
                )
                await asyncio.sleep(telemetry_interval)
        except asyncio.CancelledError:
            self.logger.warning("Secondary climb phase was cancelled.")
            raise
        except Exception as e:
            self.logger.error(f"Error during secondary climb phase: {e}")
            await self.abort_transition()
            raise

    async def ramp_throttle_and_tilt(self) -> None:
        """
        Gradually ramp throttle and tilt over their respective configured durations.
        """
        self.fwd_transition_start_time = asyncio.get_event_loop().time()
        self.logger.info(f"Starting throttle and tilt ramping at {self.fwd_transition_start_time}")
        throttle_ramp_time = self.config.get("throttle_ramp_time", 5.0)
        tilt_ramp_time = self.config.get("forward_transition_time", 15.0)
        telemetry_interval = self.config.get("telemetry_update_interval", 0.1)

        max_throttle = self.config.get("max_throttle", 0.8)
        max_tilt = -self.config.get("max_tilt_pitch", 80.0)  # Negative for downward tilt
        transition_yaw_angle = self.config.get("transition_yaw_angle", 0.0)

        throttle_steps = int(throttle_ramp_time / telemetry_interval)
        tilt_steps = int(tilt_ramp_time / telemetry_interval)

        # Retrieve initial throttle from telemetry
        telemetry = self.telemetry_handler.get_telemetry()
        fixedwing_metrics = telemetry.get("fixedwing_metrics")
        current_throttle = fixedwing_metrics.throttle_percentage if fixedwing_metrics else 0.7

        throttle_step = (max_throttle - current_throttle) / throttle_steps if throttle_steps > 0 else 0
        tilt_step = max_tilt / tilt_steps if tilt_steps > 0 else 0

        throttle = current_throttle
        tilt = 0.0

        self.logger.info(
            f"Ramping throttle from {throttle:.2f} to {max_throttle} over {throttle_ramp_time} seconds."
        )
        self.logger.info(
            f"Ramping tilt from {tilt:.0f}° to {max_tilt}° over {tilt_ramp_time} seconds."
        )

        try:
            for step in range(max(throttle_steps, tilt_steps)):
                # Check if abort event is set
                if self.abort_event.is_set() or self.transition_event.is_set():
                    self.logger.info("Ramping task received abort or transition signal.")
                    break

                telemetry = self.telemetry_handler.get_telemetry()
                fixedwing_metrics = telemetry.get("fixedwing_metrics")
                euler_angle = telemetry.get("euler_angle")

                # Update throttle
                if step < throttle_steps:
                    throttle += throttle_step
                    throttle = min(throttle, max_throttle)

                # Update tilt
                if step < tilt_steps:
                    tilt += tilt_step
                    tilt = max(tilt, max_tilt)  # Ensure tilt does not exceed max (negative)

                # Send Attitude Command
                async with self.command_lock:
                    await self.drone.offboard.set_attitude(
                        Attitude(
                            roll_deg=0.0,
                            pitch_deg=tilt,
                            yaw_deg=transition_yaw_angle,
                            thrust_value=throttle  # Correct parameter name
                        )
                    )

                # Log Telemetry Data
                current_tilt_real = euler_angle.pitch_deg if euler_angle else 0.0
                current_airspeed_real = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0

                self.logger.info(
                    f"Step {step + 1}/{tilt_steps}: "
                    f"Throttle: {throttle:.2f}, "
                    f"Tilt Actual/Command: {current_tilt_real:.0f}/{tilt:.0f}, "
                    f"Airspeed: {current_airspeed_real:.0f} m/s"
                )

                await asyncio.sleep(telemetry_interval)

            self.logger.info("Throttle and tilt ramping complete.")

        except asyncio.CancelledError:
            self.logger.warning("Ramping task was cancelled.")
            raise
        except Exception as e:
            self.logger.error(f"Error during throttle and tilt ramping: {e}")
            await self.abort_transition()
            raise

    async def monitor_and_switch(self) -> str:
        """
        Monitor telemetry and transition to fixed-wing mode when criteria are met.

        :return: Status string indicating 'success' or 'failure'.
        """
        transition_timeout = self.config.get("transition_timeout", 120.0)
        transition_air_speed = self.config.get("transition_air_speed", 20.0)
        telemetry_interval = self.config.get("telemetry_update_interval", 0.1)

        self.logger.info("Starting monitoring task.")

        try:
            while True:
                # Calculate elapsed time since ramping started
                elapsed_time = asyncio.get_event_loop().time() - self.fwd_transition_start_time

                # Check for timeout
                if elapsed_time > transition_timeout:
                    self.logger.warning("Transition timeout reached. Aborting transition.")
                    self.abort_event.set()  # Signal ramping to abort
                    await self.abort_transition()
                    return "failure"

                # Retrieve telemetry
                telemetry = self.telemetry_handler.get_telemetry()
                fixedwing_metrics = telemetry.get("fixedwing_metrics")
                airspeed = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0

                self.logger.debug(f"Monitoring: Airspeed = {airspeed} m/s")

                # Check if airspeed is sufficient for transition
                if airspeed >= transition_air_speed:
                    self.logger.info("Airspeed sufficient for transition. Initiating transition to fixed-wing mode.")
                    self.transition_event.set()  # Signal ramping to stop
                    transition_status = await self.success_transition()
                    return transition_status

                await asyncio.sleep(telemetry_interval)

        except asyncio.CancelledError:
            self.logger.warning("Monitoring task was cancelled.")
            return "failure"
        except Exception as e:
            self.logger.error(f"Error during monitoring: {e}")
            await self.abort_transition()
            return "failure"

    async def success_transition(self) -> str:
        """
        Transition to fixed-wing and initiate Hold Flight Mode.

        :return: Status string indicating 'success' or 'failure'.
        """
        self.logger.info("Transition succeeded: performing fixed-wing switch.")

        try:
            async with self.command_lock:
                # Transition to fixed-wing
                await self.drone.action.transition_to_fixedwing()
            self.logger.info("Transitioned to fixed-wing mode.")

            async with self.command_lock:
                # Initiate Hold Flight Mode
                await self.drone.action.hold()
            self.logger.info("HOLD mode activated.")

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
                async with self.command_lock:
                    await self.drone.action.transition_to_multicopter()
                self.logger.info("Transitioned to multicopter mode for safety.")
        except Exception as e:
            self.logger.warning(f"Error transitioning to multicopter: {e}")

        try:
            async with self.command_lock:
                await self.drone.offboard.stop()
            self.logger.info("Offboard mode stopped.")
        except Exception as e:
            self.logger.error(f"Error stopping offboard mode: {e}")

        try:
            async with self.command_lock:
                await self.drone.action.return_to_launch()
            self.logger.info("Return to Launch initiated.")
        except Exception as e:
            self.logger.error(f"Error initiating Return to Launch: {e}")

        return "failure"
