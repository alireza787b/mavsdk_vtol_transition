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
    Handles arming, takeoff, initial climb, throttle ramping, tilt ramping with over-tilting capability,
    integrated monitoring, and failsafes.
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
        self.ramping_started_event = asyncio.Event()

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
            if self.config.get("safety_lock", True):
                self.logger.info("Safety lock is active... Stopping the Mission...")
                return # Safety Lock
            
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

            # Process completed tasks
            for task in done:
                result = task.result()
                if result == "success":
                    self.logger.info("Transition executed successfully.")
                    # Cancel the monitoring task if it's still running
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
        cycle_interval = self.config.get("cycle_interval", 0.1)

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
                await asyncio.sleep(cycle_interval)
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
        cycle_interval = self.config.get("cycle_interval", 0.1)

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
                await asyncio.sleep(cycle_interval)
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
        If 'over_tilt_enabled' is True, continue tilting beyond 'max_tilt_pitch' up to 'max_allowed_tilt' to gain additional airspeed.
        This over-tilting continues until airspeed is sufficient for transition or a failsafe condition is triggered.

        Steps:
            1. Ramp throttle from current value to 'max_throttle' over 'throttle_ramp_time'.
            2. Ramp tilt from 0° to 'max_tilt_pitch' over 'forward_transition_time'.
            3. If 'over_tilt_enabled', continue tilting from 'max_tilt_pitch' to 'max_allowed_tilt' at the same tilt rate.
        """
        self.logger.info("Starting throttle and tilt ramping.")
        self.fwd_transition_start_time = asyncio.get_event_loop().time()
        self.logger.info(f"Throttle and tilt ramping started at {self.fwd_transition_start_time:.2f}.")

        # Signal that ramping has started to prevent race conditions
        self.ramping_started_event.set()

        # Retrieve configuration parameters with defaults
        throttle_ramp_time = self.config.get("throttle_ramp_time", 5.0)  # (s)
        tilt_ramp_time = self.config.get("forward_transition_time", 15.0)  # (s)
        cycle_interval = self.config.get("cycle_interval", 0.1)  # (s)
        over_tilt_enabled = self.config.get("over_tilt_enabled", False)  # (bool)
        max_allowed_tilt = self.config.get("max_allowed_tilt", -110.0)  # (degrees, negative for downward tilt)

        max_throttle = self.config.get("max_throttle", 0.8)  # (unitless, e.g., 0.0 to 1.0)
        max_tilt = self.config.get("max_tilt_pitch", 80.0)  # (degrees, negative for downward tilt)
        transition_yaw_angle = self.config.get("transition_yaw_angle", 0.0)  # (degrees)

        # Calculate the number of steps for ramping
        throttle_steps = int(throttle_ramp_time / cycle_interval)
        tilt_steps = int(tilt_ramp_time / cycle_interval)
        total_steps = max(throttle_steps, tilt_steps)

        # Retrieve initial throttle from telemetry
        telemetry = self.telemetry_handler.get_telemetry()
        fixedwing_metrics = telemetry.get("fixedwing_metrics")
        current_throttle = fixedwing_metrics.throttle_percentage if fixedwing_metrics else 0.7  # Default throttle

        throttle_step = (max_throttle - current_throttle) / throttle_steps if throttle_steps > 0 else 0
        tilt_step = max_tilt / tilt_steps if tilt_steps > 0 else 0

        throttle = current_throttle
        tilt = 0.0  # Starting tilt

        self.logger.info(
            f"Ramping throttle from {throttle:.2f} to {max_throttle:.2f} over {throttle_ramp_time} seconds."
        )
        self.logger.info(
            f"Ramping tilt from {tilt:.0f}° to {max_tilt:.0f}° over {tilt_ramp_time} seconds."
        )
        if over_tilt_enabled:
            self.logger.info(
                f"Over-tilting enabled. Will continue tilting up to {max_allowed_tilt:.0f}° if necessary."
            )

        try:
            # Phase 1: Initial Ramping
            for step in range(total_steps):
                # Check if abort or transition event is set
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
                    f"Step {step + 1}/{total_steps}: "
                    f"Throttle: {throttle:.2f}, "
                    f"Tilt Actual/Command: {current_tilt_real:.0f}/{tilt:.0f}°, "
                    f"Airspeed: {current_airspeed_real:.0f} m/s"
                )

                await asyncio.sleep(cycle_interval)

            # Phase 2: Over-Tilting (if enabled)
            if over_tilt_enabled and tilt > max_tilt:
                self.logger.info("Initiating over-tilting phase.")
                # Calculate remaining tilt steps to reach max_allowed_tilt
                additional_tilt = max_allowed_tilt - tilt
                if tilt_step != 0:
                    over_tilt_steps = int(abs(additional_tilt) / abs(tilt_step))
                else:
                    over_tilt_steps = 0

                self.logger.info(
                    f"Ramping tilt from {tilt:.0f}° to {max_allowed_tilt:.0f}° over {over_tilt_steps * cycle_interval:.1f} seconds."
                )

                for step in range(over_tilt_steps):
                    # Check if abort or transition event is set
                    if self.abort_event.is_set() or self.transition_event.is_set():
                        self.logger.info("Over-tilting task received abort or transition signal.")
                        break

                    telemetry = self.telemetry_handler.get_telemetry()
                    fixedwing_metrics = telemetry.get("fixedwing_metrics")
                    euler_angle = telemetry.get("euler_angle")

                    # Update tilt
                    tilt += tilt_step
                    tilt = max(tilt, max_allowed_tilt)  # Ensure tilt does not exceed max_allowed_tilt (negative)

                    # Send Attitude Command
                    async with self.command_lock:
                        await self.drone.offboard.set_attitude(
                            Attitude(
                                roll_deg=0.0,
                                pitch_deg=tilt,
                                yaw_deg=transition_yaw_angle,
                                thrust_value=throttle  # Throttle remains at max
                            )
                        )

                    # Log Telemetry Data
                    current_tilt_real = euler_angle.pitch_deg if euler_angle else 0.0
                    current_airspeed_real = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0

                    self.logger.info(
                        f"Over-Tilt Step {step + 1}/{over_tilt_steps}: "
                        f"Tilt Actual/Command: {current_tilt_real:.0f}/{tilt:.0f}°, "
                        f"Airspeed: {current_airspeed_real:.0f} m/s"
                    )

                    await asyncio.sleep(cycle_interval)

                self.logger.info("Over-tilting phase complete.")

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
        Incorporates additional failsafe conditions:
            - Roll angle exceeds +/- max_roll_failsafe
            - Altitude exceeds max_altitude_failsafe
            - Pitch angle exceeds max_pitch_failsafe
            - Altitude loss exceeds altitude_loss_limit
            - Altitude falls below altitude_failsafe_threshold
            - Climb rate falls below climb_rate_failsafe_threshold
            - Transition timeout exceeded
        :return: Status string indicating 'success' or 'failure'.
        """
        # Retrieve configuration parameters with defaults
        transition_timeout = self.config.get("transition_timeout", 120.0)  # (s)
        transition_air_speed = self.config.get("transition_air_speed", 20.0)  # (m/s)
        cycle_interval = self.config.get("cycle_interval", 0.1)  # (s)

        # Additional failsafe parameters
        max_roll_failsafe = self.config.get("max_roll_failsafe", 30.0)  # (degrees)
        max_altitude_failsafe = self.config.get("max_altitude_failsafe", 200.0)  # (m)
        max_pitch_failsafe = self.config.get("max_pitch_failsafe", 130.0)  # (degrees)
        altitude_loss_limit = self.config.get("altitude_loss_limit", 20.0)  # (m)
        altitude_failsafe_threshold = self.config.get("altitude_failsafe_threshold", 10.0)  # (m)
        climb_rate_failsafe_threshold = self.config.get("climb_rate_failsafe_threshold", 0.3)  # (m/s)

        self.logger.info("Starting monitoring task with additional failsafe conditions.")

        # Initialize maximum altitude tracking
        max_altitude = None

        try:
            while True:
                # Ensure that ramping has started before proceeding
                if not self.ramping_started_event.is_set():
                    self.logger.debug("Waiting for ramping to start...")
                    await self.ramping_started_event.wait()

                # Calculate elapsed time since ramping started
                elapsed_time = asyncio.get_event_loop().time() - self.fwd_transition_start_time

                # Retrieve telemetry data
                telemetry = self.telemetry_handler.get_telemetry()
                position_velocity_ned = telemetry.get("position_velocity_ned")
                euler_angle = telemetry.get("euler_angle")
                fixedwing_metrics = telemetry.get("fixedwing_metrics")

                # Extract necessary telemetry information
                altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0
                pitch = euler_angle.pitch_deg if euler_angle else 0.0
                roll = euler_angle.roll_deg if euler_angle else 0.0
                airspeed = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0
                climb_rate = position_velocity_ned.climb_rate_m_s if position_velocity_ned else 0.0

                # Update maximum altitude achieved
                if max_altitude is None or altitude > max_altitude:
                    max_altitude = altitude
                    self.logger.debug(f"Updated max_altitude to {max_altitude:.2f} meters.")

                # Calculate altitude loss since reaching maximum altitude
                altitude_loss = max_altitude - altitude if max_altitude else 0.0
                self.logger.debug(
                    f"Telemetry - Altitude: {altitude:.2f} m, "
                    f"Max Altitude: {max_altitude:.2f} m, "
                    f"Altitude Loss: {altitude_loss:.2f} m, "
                    f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, "
                    f"Airspeed: {airspeed:.2f} m/s, Climb Rate: {climb_rate:.2f} m/s"
                )

                # 1. Check Roll Angle Failsafe
                if abs(roll) > max_roll_failsafe:
                    self.logger.warning(
                        f"Roll angle exceeded failsafe threshold: {roll:.2f}° > ±{max_roll_failsafe}°."
                    )
                    self.abort_event.set()  # Signal ramping to abort
                    await self.abort_transition()
                    return "failure"

                # 2. Check Max Altitude Failsafe
                if altitude > max_altitude_failsafe:
                    self.logger.warning(
                        f"Altitude exceeded failsafe threshold: {altitude:.2f} m > {max_altitude_failsafe} m."
                    )
                    self.abort_event.set()  # Signal ramping to abort
                    await self.abort_transition()
                    return "failure"

                # 3. Check Pitch Angle Failsafe
                if abs(pitch) > max_pitch_failsafe:
                    self.logger.warning(
                        f"Pitch angle exceeded failsafe threshold: {pitch:.2f}° > {max_pitch_failsafe}°."
                    )
                    self.abort_event.set()  # Signal ramping to abort
                    await self.abort_transition()
                    return "failure"

                # 4. Check Altitude Loss Limit
                if altitude_loss > altitude_loss_limit:
                    self.logger.warning(
                        f"Altitude loss exceeded limit: {altitude_loss:.2f} m > {altitude_loss_limit} m."
                    )
                    self.abort_event.set()  # Signal ramping to abort
                    await self.abort_transition()
                    return "failure"

                # 5. Check Altitude Failsafe Threshold
                if altitude < altitude_failsafe_threshold:
                    self.logger.warning(
                        f"Altitude below failsafe threshold: {altitude:.2f} m < {altitude_failsafe_threshold} m."
                    )
                    self.abort_event.set()  # Signal ramping to abort
                    await self.abort_transition()
                    return "failure"

                # 6. Check Climb Rate Failsafe Threshold
                if climb_rate < climb_rate_failsafe_threshold:
                    self.logger.warning(
                        f"Climb rate below failsafe threshold: {climb_rate:.2f} m/s < {climb_rate_failsafe_threshold} m/s."
                    )
                    self.abort_event.set()  # Signal ramping to abort
                    await self.abort_transition()
                    return "failure"

                # Check if airspeed is sufficient for transition
                if airspeed >= transition_air_speed:
                    self.logger.info(
                        f"Airspeed sufficient for transition: {airspeed:.2f} m/s >= {transition_air_speed} m/s."
                    )
                    self.transition_event.set()  # Signal ramping to stop
                    transition_status = await self.success_transition()
                    return transition_status

                # Check for transition timeout
                if elapsed_time > transition_timeout:
                    self.logger.warning(
                        f"Transition timeout reached: {elapsed_time:.2f} s > {transition_timeout} s. Aborting transition."
                    )
                    self.abort_event.set()  # Signal ramping to abort
                    await self.abort_transition()
                    return "failure"

                # Optional: Additional telemetry logging for debugging
                self.logger.debug(
                    f"Telemetry - Altitude: {altitude:.2f} m, "
                    f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, "
                    f"Airspeed: {airspeed:.2f} m/s, Climb Rate: {climb_rate:.2f} m/s, "
                    f"Elapsed Time: {elapsed_time:.2f} s."
                )

                await asyncio.sleep(cycle_interval)

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
