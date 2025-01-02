#modules/transition_logic/tailsitter_pitch_program.py

import asyncio
import logging
from mavsdk.offboard import (
    VelocityBodyYawspeed,
    VelocityNedYaw,
    Attitude,
    AttitudeRate,
    OffboardError,
)
from mavsdk.mission import MissionError
from modules.transition_logic.post_transition_actions import PostTransitionAction

# Optionally, define an enum for your post-transition actions. If you prefer
# to keep it simple with just strings in the config, you can skip the enum.
#
# from enum import Enum
# class PostTransitionAction(Enum):
#     CONTINUE_CURRENT_HEADING = "continue_current_heading"
#     HOLD = "hold"
#     RETURN_TO_LAUNCH = "return_to_launch"
#     START_MISSION_FROM_WAYPOINT = "start_mission_from_waypoint"


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
        Orchestrates:
          - Arm + Takeoff
          - Switch to Offboard
          - Initial and Secondary Climb
          - Throttle & Tilt Ramping + Monitoring
        Returns 'success' or 'failure'.
        """
        self.logger.info("Starting VTOL transition program.")
        try:
            # Phase 1: Arm and Takeoff
            if self.config.get("safety_lock", True):
                self.logger.info("Safety lock is active... Stopping the Mission...")
                return  # Safety Lock

            await self.arm_and_takeoff()

            # Phase 2: Enter Offboard Mode
            await self.start_offboard(retries=3)

            # Phase 3: Initial Climb
            await self.initial_climb_phase()

            # Phase 4: Secondary Climb
            await self.secondary_climb_phase()

            # Phase 5: Ramping and Monitoring (run concurrently)
            ramping_task = asyncio.create_task(self.ramp_throttle_and_tilt())
            monitoring_task = asyncio.create_task(self.monitor_and_switch())

            done, pending = await asyncio.wait(
                [ramping_task, monitoring_task],
                return_when=asyncio.FIRST_COMPLETED
            )

            # Process whichever task finished first
            for task in done:
                result = task.result()
                if result == "success":
                    self.logger.info("Transition executed successfully.")
                    # Cancel the other task(s)
                    for pending_task in pending:
                        pending_task.cancel()
                    return "success"
                elif result == "failure":
                    self.logger.warning("Transition failed during monitoring/ramping.")
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
        Phase 1: Arm the drone and initiate takeoff.
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
            await asyncio.sleep(5)  # Wait a few seconds for the drone to stabilize
        except Exception as e:
            self.logger.error(f"Takeoff failed: {e}")
            await self.abort_transition()
            raise

    async def start_offboard(self, retries: int = 3) -> None:
        """
        Phase 2: Enter offboard mode with a configurable number of retries.
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
        Phase 3: Initial climb to a preliminary altitude.
        """
        initial_climb_height = self.config.get("initial_climb_height", 5.0)
        initial_climb_rate = self.config.get("initial_climb_rate", 2.0)
        cycle_interval = self.config.get("cycle_interval", 0.1)

        self.logger.info(
            f"Starting initial climb to {initial_climb_height}m at {initial_climb_rate}m/s."
        )

        try:
            while True:
                telemetry = self.telemetry_handler.get_telemetry()
                position_velocity_ned = telemetry.get("position_velocity_ned")
                altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0

                if altitude >= initial_climb_height:
                    self.logger.info(f"Reached initial climb height: {altitude:.2f}m.")
                    break

                # Positive Body Z is downward => negative velocity for upward movement
                async with self.command_lock:
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, -initial_climb_rate, 0.0)
                    )

                self.logger.info(
                    f"Initial climb in progress... Alt: {altitude:.2f}m, Target: {initial_climb_height}m."
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
        Phase 4: Secondary climb to the transition base altitude.
        """
        transition_base_altitude = self.config.get("transition_base_altitude", 10.0)
        secondary_climb_rate = self.config.get("secondary_climb_rate", 1.0)
        transition_yaw_angle = self.config.get("transition_yaw_angle", 0.0)
        cycle_interval = self.config.get("cycle_interval", 0.1)

        self.logger.info(
            f"Starting secondary climb to {transition_base_altitude}m at {secondary_climb_rate}m/s."
        )

        try:
            while True:
                telemetry = self.telemetry_handler.get_telemetry()
                position_velocity_ned = telemetry.get("position_velocity_ned")
                altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0

                if altitude >= transition_base_altitude:
                    self.logger.info(f"Reached transition base altitude: {altitude:.2f}m.")
                    break

                # Upward velocity in NED (down = positive)
                async with self.command_lock:
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, -secondary_climb_rate, transition_yaw_angle)
                    )

                self.logger.info(
                    f"Secondary climb in progress... Alt: {altitude:.2f}m, Target: {transition_base_altitude}m."
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
        Phase 5a: Gradually ramp throttle and tilt, with optional 'over-tilt' capability.
        """
        self.logger.info("Starting throttle and tilt ramping.")
        self.fwd_transition_start_time = asyncio.get_event_loop().time()
        self.logger.info(f"Throttle and tilt ramping started at {self.fwd_transition_start_time:.2f}.")

        # Signal that ramping has started (prevents race conditions in monitoring)
        self.ramping_started_event.set()

        # Read config parameters
        throttle_ramp_time = self.config.get("throttle_ramp_time", 5.0)     # seconds
        tilt_ramp_time = self.config.get("forward_transition_time", 15.0)   # seconds
        cycle_interval = self.config.get("cycle_interval", 0.1)            # seconds

        over_tilt_enabled = self.config.get("over_tilt_enabled", False)
        max_allowed_tilt = -1 * self.config.get("max_allowed_tilt", 110.0)   # negative degrees
        max_throttle = self.config.get("max_throttle", 0.8)
        max_tilt = -1 * self.config.get("max_tilt_pitch", 80.0)
        transition_yaw_angle = self.config.get("transition_yaw_angle", 0.0)

        # Calculate ramping step counts
        throttle_steps = int(throttle_ramp_time / cycle_interval)
        tilt_steps = int(tilt_ramp_time / cycle_interval)
        total_steps = max(throttle_steps, tilt_steps)

        # Retrieve current throttle from telemetry; default to 0.7 if missing
        telemetry = self.telemetry_handler.get_telemetry()
        fixedwing_metrics = telemetry.get("fixedwing_metrics")
        current_throttle = fixedwing_metrics.throttle_percentage if fixedwing_metrics else 0.7

        throttle_step = ((max_throttle - current_throttle) / throttle_steps) if throttle_steps > 0 else 0
        tilt_step = (max_tilt / tilt_steps) if tilt_steps > 0 else 0

        throttle = current_throttle
        tilt = 0.0  # initial tilt is 0 deg

        self.logger.info(
            f"Ramping throttle from {throttle:.2f} to {max_throttle:.2f} over {throttle_ramp_time:.1f}s."
        )
        self.logger.info(
            f"Ramping tilt from {tilt:.0f}° to {max_tilt:.0f}° over {tilt_ramp_time:.1f}s."
        )
        if over_tilt_enabled:
            self.logger.info(
                f"Over-tilting enabled. Will continue tilting up to {max_allowed_tilt:.0f}° if needed."
            )

        try:
            # --- Phase 1: Normal ramping ---
            for step in range(total_steps):
                # Check if we got an abort or if the transition completed
                if self.abort_event.is_set() or self.transition_event.is_set():
                    self.logger.info("Ramping task received abort/transition signal.")
                    break

                # Re-read telemetry each cycle
                telemetry = self.telemetry_handler.get_telemetry()
                fixedwing_metrics = telemetry.get("fixedwing_metrics")
                euler_angle = telemetry.get("euler_angle")
                position_velocity_ned = telemetry.get("position_velocity_ned")

                # Update throttle
                if step < throttle_steps:
                    throttle += throttle_step
                    throttle = min(throttle, max_throttle)

                # Update tilt
                if step < tilt_steps:
                    tilt += tilt_step
                    tilt = max(tilt, max_tilt)  # tilt is negative => "max()" is the more negative

                # Send Attitude Command
                async with self.command_lock:
                    await self.drone.offboard.set_attitude(
                        Attitude(
                            roll_deg=0.0,
                            pitch_deg=tilt,
                            yaw_deg=transition_yaw_angle,
                            thrust_value=throttle
                        )
                    )

                # Logging
                current_tilt_real = euler_angle.pitch_deg if euler_angle else 0.0
                current_airspeed_real = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0
                current_altitude_real = (
                    -position_velocity_ned.position.down_m
                    if position_velocity_ned else 0.0
                )
                self.logger.info(
                    f"Step {step + 1}/{total_steps} | "
                    f"Throttle: {throttle:.2f}, TiltCmd/Actual: {tilt:.0f}/{current_tilt_real:.0f}°, "
                    f"Airspeed: {current_airspeed_real:.1f}m/s, Alt: {current_altitude_real:.1f}m"
                )

                await asyncio.sleep(cycle_interval)

            # --- Phase 2: Over-Tilting (if enabled) ---
            if over_tilt_enabled and tilt > max_tilt:
                self.logger.info("Initiating over-tilting phase.")
                additional_tilt = max_allowed_tilt - tilt
                over_tilt_steps = (
                    int(additional_tilt / tilt_step) if tilt_step != 0 else 0
                )
                self.logger.info(
                    f"Over-tilting from {tilt:.0f}° to {max_allowed_tilt:.0f}° "
                    f"in ~{over_tilt_steps * cycle_interval:.1f}s."
                )

                for step in range(over_tilt_steps):
                    if self.abort_event.is_set() or self.transition_event.is_set():
                        self.logger.info("Over-tilting task received abort/transition signal.")
                        break

                    telemetry = self.telemetry_handler.get_telemetry()
                    fixedwing_metrics = telemetry.get("fixedwing_metrics")
                    euler_angle = telemetry.get("euler_angle")

                    tilt += tilt_step
                    tilt = max(tilt, max_allowed_tilt)  # tilt is negative => "max()" is more negative

                    async with self.command_lock:
                        await self.drone.offboard.set_attitude(
                            Attitude(
                                roll_deg=0.0,
                                pitch_deg=tilt,
                                yaw_deg=transition_yaw_angle,
                                thrust_value=throttle  # remains at max
                            )
                        )

                    current_tilt_real = euler_angle.pitch_deg if euler_angle else 0.0
                    current_airspeed_real = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0

                    self.logger.info(
                        f"Over-Tilt Step {step + 1}/{over_tilt_steps} | "
                        f"TiltCmd/Actual: {tilt:.0f}/{current_tilt_real:.0f}°, "
                        f"Airspeed: {current_airspeed_real:.1f}m/s"
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
        Phase 5b: Monitor telemetry & failsafes. 
        Transition to FW mode when conditions are met or fail if conditions are violated.
        Returns 'success' or 'failure'.
        """
        # Config parameters
        transition_timeout = self.config.get("transition_timeout", 120.0)
        transition_air_speed = self.config.get("transition_air_speed", 20.0)
        cycle_interval = self.config.get("cycle_interval", 0.1)

        # Failsafes
        max_roll_failsafe = self.config.get("max_roll_failsafe", 30.0)
        max_altitude_failsafe = self.config.get("max_altitude_failsafe", 200.0)
        max_pitch_failsafe = self.config.get("max_pitch_failsafe", 130.0)
        altitude_loss_limit = self.config.get("altitude_loss_limit", 20.0)
        altitude_failsafe_threshold = self.config.get("altitude_failsafe_threshold", 10.0)
        climb_rate_failsafe_threshold = self.config.get("climb_rate_failsafe_threshold", 0.3)

        self.logger.info("Starting monitoring task with additional failsafe conditions.")

        # Track max altitude to detect altitude loss
        max_altitude = None

        try:
            while True:
                # Wait until ramping actually starts
                if not self.ramping_started_event.is_set():
                    self.logger.debug("Waiting for ramping to start...")
                    await self.ramping_started_event.wait()

                # Elapsed time
                elapsed_time = asyncio.get_event_loop().time() - self.fwd_transition_start_time

                # Telemetry
                telemetry = self.telemetry_handler.get_telemetry()
                position_velocity_ned = telemetry.get("position_velocity_ned")
                euler_angle = telemetry.get("euler_angle")
                fixedwing_metrics = telemetry.get("fixedwing_metrics")

                # Parse data
                altitude = -position_velocity_ned.position.down_m if position_velocity_ned else 0.0
                pitch = euler_angle.pitch_deg if euler_angle else 0.0
                roll = euler_angle.roll_deg if euler_angle else 0.0
                airspeed = fixedwing_metrics.airspeed_m_s if fixedwing_metrics else 0.0
                climb_rate = fixedwing_metrics.climb_rate_m_s if fixedwing_metrics else 0.0

                # Track highest altitude
                if max_altitude is None or altitude > max_altitude:
                    max_altitude = altitude

                altitude_loss = (max_altitude - altitude) if max_altitude else 0.0

                self.logger.debug(
                    f"Telemetry - Alt: {altitude:.2f}m, MaxAlt: {max_altitude:.2f}m, "
                    f"Loss: {altitude_loss:.2f}m, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, "
                    f"Airspeed: {airspeed:.2f}m/s, Climb: {climb_rate:.2f}m/s, "
                    f"Time: {elapsed_time:.1f}s."
                )

                # Check Failsafes
                if abs(roll) > max_roll_failsafe:
                    self.logger.warning(f"Roll exceeded failsafe: {roll:.2f}° > ±{max_roll_failsafe}°.")
                    self.abort_event.set()
                    await self.abort_transition()
                    return "failure"

                if altitude > max_altitude_failsafe:
                    self.logger.warning(f"Altitude exceeded failsafe: {altitude:.2f}m > {max_altitude_failsafe}m.")
                    self.abort_event.set()
                    await self.abort_transition()
                    return "failure"

                if abs(pitch) > max_pitch_failsafe:
                    self.logger.warning(f"Pitch exceeded failsafe: {pitch:.2f}° > {max_pitch_failsafe}°.")
                    self.abort_event.set()
                    await self.abort_transition()
                    return "failure"

                if altitude_loss > altitude_loss_limit:
                    self.logger.warning(
                        f"Altitude loss exceeded limit: {altitude_loss:.2f}m > {altitude_loss_limit}m."
                    )
                    self.abort_event.set()
                    await self.abort_transition()
                    return "failure"

                if altitude < altitude_failsafe_threshold:
                    self.logger.warning(
                        f"Altitude below failsafe threshold: {altitude:.2f}m < {altitude_failsafe_threshold}m."
                    )
                    self.abort_event.set()
                    await self.abort_transition()
                    return "failure"

                if climb_rate < climb_rate_failsafe_threshold:
                    self.logger.warning(
                        f"Climb rate below failsafe: {climb_rate:.2f}m/s < {climb_rate_failsafe_threshold}m/s."
                    )
                    self.abort_event.set()
                    await self.abort_transition()
                    return "failure"

                # Check if we have enough airspeed to transition
                if airspeed >= transition_air_speed:
                    self.logger.info(
                        f"Airspeed sufficient for transition: {airspeed:.2f}m/s >= {transition_air_speed}m/s."
                    )
                    self.transition_event.set()
                    transition_status = await self.success_transition()
                    return transition_status

                # Timeout
                if elapsed_time > transition_timeout:
                    self.logger.warning(
                        f"Transition timeout: {elapsed_time:.2f}s > {transition_timeout}s. Aborting."
                    )
                    self.abort_event.set()
                    await self.abort_transition()
                    return "failure"

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
        Called when the transition to fixed-wing is determined to be 'successful'.
        Handles:
          1) Transition to fixed-wing
          2) Accelerating to target velocity (via Body velocity)
          3) Calls the post-transition action handler
        Returns 'success' or 'failure'.
        """
        telemetry = self.telemetry_handler.get_telemetry()
        position_velocity_ned = telemetry.get("position_velocity_ned")

        fixedwing_metrics = telemetry.get("fixedwing_metrics")
        current_throttle = fixedwing_metrics.throttle_percentage if fixedwing_metrics else 0.7

        # Calculate current horizontal velocity
        if position_velocity_ned:
            vx = position_velocity_ned.velocity.north_m_s
            vy = position_velocity_ned.velocity.east_m_s
            horizontal_velocity = (vx**2 + vy**2) ** 0.5
        else:
            horizontal_velocity = self.config.get("transition_air_speed", 20.0)

        # Acceleration factor
        acceleration_factor = self.config.get("acceleration_factor", 1.0)
        target_horizontal_velocity = horizontal_velocity * acceleration_factor

        self.logger.info(f"Current Horizontal Velocity: {horizontal_velocity:.2f} m/s")
        self.logger.info(
            f"Target Horizontal Velocity after applying accel factor {acceleration_factor}: "
            f"{target_horizontal_velocity:.2f} m/s"
        )

        #TODO: add like last vresion intiall body accelrateon
        self.logger.info("Transition succeeded: performing fixed-wing switch.")
        
        
        # Methode 1
        # Accelerate in Body frame before transition 
        #   e.g., forward in the x direction
        async with self.command_lock:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(target_horizontal_velocity, 0.0, 0.0, 0.0)
            )
        self.logger.info("Accelerating to cruise airspeed in offboard mode before transition.")

        # Optionally wait a short time to let the drone accelerate
        await asyncio.sleep(self.config.get("acceleration_duration", 0.5))

        # Methode 2
        # Hold Attitude before transition
        # async with self.command_lock:
        #                 await self.drone.offboard.set_attitude_rate(
        #                     AttitudeRate(
        #                         roll_deg_s=0.0,
        #                         pitch_deg_s=0.0,
        #                         yaw_deg_s=0.0,
        #                         thrust_value=current_throttle 
        #                     )
        #                 )
                        
                
        # Stop offboard if you don't want to remain in it
            # (If you want to stay in offboard, you can comment out the following lines)
        try:
            async with self.command_lock:
                            await self.drone.offboard.stop()
            self.logger.info("Offboard mode stopped after acceleration phase.")
        except Exception as e:
            self.logger.error(f"Error stopping offboard mode: {e}")
                        
        
        try:
            async with self.command_lock:
                # Transition to fixed-wing
                await self.drone.action.transition_to_fixedwing()
            self.logger.info("Transitioned to fixed-wing mode.")


            

            # Next: handle post-transition action (modular)
            await self.handle_post_transition_action()

            return "success"

        except Exception as e:
            self.logger.error(f"Error during fixed-wing transition: {e}")
            await self.abort_transition()
            return "failure"

    async def handle_post_transition_action(self) -> None:
        """
        A modular handler for what to do immediately after successful transition to FW.
        Reads 'post_transition_action' from config (string).
        Options might include:
          - continue_current_heading
          - hold
          - return_to_launch
          - start_mission_from_waypoint
        """
        action_name = self.config.get("post_transition_action", "return_to_launch").lower()
        self.logger.info(f"Post-transition action requested: '{action_name}'")

        try:
            if action_name == PostTransitionAction.CONTINUE_CURRENT_HEADING.value:
                await self._continue_current_heading()

            elif action_name == PostTransitionAction.HOLD.value:
                await self._hold_mode()

            elif action_name == PostTransitionAction.START_MISSION_FROM_WAYPOINT.value:
                # TODO You might also read a 'waypoint_index' from config
                waypoint_index = self.config.get("start_waypoint_index", 2)
                await self._start_mission_from_waypoint(waypoint_index)

            #Default or explicit: return_to_launch
            else:
                self.logger.info("Executing RETURN_TO_LAUNCH as post-transition action.")
                await self._return_to_launch()

        except Exception as e:
            # If post-transition action fails, you can decide to abort or just log an error.
            self.logger.error(f"Post-transition action failed: {e}")
            await self._return_to_launch()
            self.logger.info("Executing RETURN_TO_LAUNCH as a fallback failsafe action.")
            

    async def _continue_current_heading(self) -> None:
        """
        Command the drone in Offboard to keep flying forward on its current heading.
        This is an example method you can adapt to your needs.
        """
        self.logger.info("Continuing on current heading with offboard velocity setpoints.")

        await self.start_offboard()
        self.logger.info("Offfboard mode started again... Continuing on current heading with offboard velocity setpoints.")

        # Example: read current velocities from telemetry
        telemetry = self.telemetry_handler.get_telemetry()
        position_velocity_ned = telemetry.get("position_velocity_ned")
        euler_angle = telemetry.get("euler_angle")
        current_yaw = euler_angle.yaw_deg
        transition_air_speed = self.config.get("transition_air_speed", 20.0)
        position_velocity_ned = telemetry.get("position_velocity_ned")
        if not position_velocity_ned:
            self.logger.warning("No position_velocity_ned; defaulting forward velocity to transition airspeed in body.")
            vel_fwd, vel_right, vel_d = transition_air_speed, 0.0, 0.0
            yaw_rate_deg = 0
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vel_fwd, vel_right, vel_d, yaw_rate_deg)
            )

            self.logger.info(
                f"Set velocity Body to FWD:{vel_right:.2f} E:{vel_right:.2f} D:{vel_d:.2f}, yaw:{yaw_rate_deg:.1f}°."
            )
            
        else:
            vel_n = position_velocity_ned.velocity.north_m_s
            vel_e = position_velocity_ned.velocity.east_m_s
            # Zero vertical speed => maintain altitude
            vel_d = 0.0
            # Yaw could be derived from arctan2(vel_e, vel_n), or just set to 0
            yaw_deg = current_yaw

        async with self.command_lock:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(vel_n, vel_e, vel_d, yaw_deg)
            )

        self.logger.info(
            f"Set velocity NED to N:{vel_n:.2f} E:{vel_e:.2f} D:{vel_d:.2f}, yaw:{yaw_deg:.1f}°."
        )
        # Up to you whether to remain in offboard or switch to another mode after some time

    async def _hold_mode(self) -> None:
        """
        Switch the drone to HOLD flight mode.
        """
        self.logger.info("Switching drone to HOLD flight mode.")
        async with self.command_lock:
            await self.drone.action.hold()
        self.logger.info("HOLD mode activated.")

    async def _return_to_launch(self) -> None:
        """
        Command the drone to Return to Launch (RTL).
        """
        self.logger.info("Initiating Return to Launch.")
        async with self.command_lock:
            await self.drone.action.return_to_launch()
        self.logger.info("Return to Launch activated.")

    async def _start_mission_from_waypoint(self, waypoint_index: int) -> None:
        """
        Retrieves the current mission from the drone, logs the mission details,
        sets the specified mission waypoint as current, and starts the mission.

        Parameters:
            waypoint_index (int): The 0-based index of the mission waypoint to start from.
        """
        # Retrieve and log the current mission plan
        try:
            async with self.command_lock:
                mission_plan = await self.drone.mission.download_mission()
            self.logger.info("Retrieved current mission plan from the drone:")
            for idx, item in enumerate(mission_plan.mission_items):
                self.logger.info(
                    f"  Waypoint {idx}: "
                    f"Lat={item.latitude_deg}, Lon={item.longitude_deg}, Alt={item.relative_altitude_m}m, "
                    f"Speed={item.speed_m_s}m/s, Fly-Through={'Yes' if item.is_fly_through else 'No'}, "
                    f"Yaw={item.yaw_deg}°, Camera Action={item.camera_action}, "
                    f"Vehicle Action={item.vehicle_action}"
                )
        except MissionError as e:
            self.logger.error(f"Failed to download mission: {e}")
            # return
        except Exception as e:
            self.logger.error(f"Unexpected error while downloading mission: {e}")
            # return

        # Validate the waypoint index
        if not (0 <= waypoint_index < len(mission_plan.mission_items)):
            self.logger.error(
                f"Invalid waypoint index: {waypoint_index}. "
                f"Mission has {len(mission_plan.mission_items)} waypoints."
            )
            # return

        # Set the specified mission waypoint as current
        self.logger.info(
            f"Setting mission item to waypoint #{waypoint_index} and preparing to start mission."
        )
        try:
            async with self.command_lock:
                await self.drone.mission.set_current_mission_item(waypoint_index)
            self.logger.info(f"Current mission item set to waypoint index: {waypoint_index}.")
        except MissionError as e:
            self.logger.error(f"Failed to set mission item to waypoint {waypoint_index}: {e}")
            # return
        except Exception as e:
            self.logger.error(f"Unexpected error while setting mission item: {e}")
            # return

        # Start the mission
        try:
            async with self.command_lock:
                await self.drone.mission.start_mission()
            self.logger.info("Mission started successfully.")
        except MissionError as e:
            self.logger.error(f"Failed to start mission: {e}")
        except Exception as e:
            self.logger.error(f"Unexpected error while starting mission: {e}")


    async def abort_transition(self) -> str:
        """
        Abort the transition and ensure the drone switches to a safe state.
        Returns 'failure'.
        """
        self.logger.error("Aborting transition and initiating fail-safe procedures.")

        # Attempt transition to multicopter if configured
        try:
            if self.config.get("failsafe_multicopter_transition", True):
                async with self.command_lock:
                    await self.drone.action.transition_to_multicopter()
                self.logger.info("Transitioned to multicopter mode for safety.")
        except Exception as e:
            self.logger.warning(f"Error transitioning to multicopter: {e}")

        # Stop offboard
        try:
            async with self.command_lock:
                await self.drone.offboard.stop()
            self.logger.info("Offboard mode stopped.")
        except Exception as e:
            self.logger.error(f"Error stopping offboard mode: {e}")

        # Return to Launch as a final fallback
        try:
            async with self.command_lock:
                await self.drone.action.return_to_launch()
            self.logger.info("Return to Launch initiated for fail-safe.")
        except Exception as e:
            self.logger.error(f"Error initiating Return to Launch: {e}")

        return "failure"
