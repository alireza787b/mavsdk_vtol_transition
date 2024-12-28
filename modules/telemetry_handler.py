# modules/telemetry_handler.py

import asyncio
import logging
from mavsdk import System
from mavsdk.telemetry import Battery, FixedwingMetrics, EulerAngle, PositionNed
from rich.console import Console
from rich.table import Table

class TelemetryHandler:
    """
    Handles telemetry data retrieval and logging.
    Manages telemetry subscriptions and provides access to the latest telemetry data.
    """

    def __init__(self, drone: System, config: dict, verbose: bool = False):
        """
        Initialize the TelemetryHandler with drone instance and configuration.

        :param drone: MAVSDK System instance representing the drone.
        :param config: Configuration dictionary containing telemetry parameters.
        :param verbose: Flag to enable detailed telemetry display.
        """
        self.drone = drone
        self.update_interval = config.get('telemetry_update_interval', 1.0)
        self.verbose = verbose
        self.logger = logging.getLogger(self.__class__.__name__)
        self.telemetry_data = {}
        self.console = Console()
        self.subscriptions = []  # List to keep track of telemetry subscription tasks

    async def start_telemetry(self) -> None:
        """
        Initializes telemetry subscriptions based on verbosity.
        Starts all telemetry subscription tasks and stores them for management.
        """
        self.logger.info("Starting telemetry subscriptions...")

        # Start each telemetry subscription as a separate asyncio Task
        self.subscriptions = [
            asyncio.create_task(self.subscribe_battery()),
            asyncio.create_task(self.subscribe_fixedwing_metrics()),
            asyncio.create_task(self.subscribe_euler_angle()),
            asyncio.create_task(self.subscribe_position_ned())
        ]

    async def subscribe_battery(self) -> None:
        """
        Subscribes to battery telemetry with error handling.
        Updates telemetry data and displays it if verbose mode is enabled.
        """
        self.logger.debug("Subscribing to battery telemetry...")
        try:
            async for battery in self.drone.telemetry.battery():
                self.telemetry_data['battery'] = battery
                await self.display_telemetry()
        except asyncio.CancelledError:
            self.logger.info("Battery telemetry subscription cancelled.")
        except Exception as e:
            self.logger.error(f"Error in battery telemetry subscription: {e}")

    async def subscribe_fixedwing_metrics(self) -> None:
        """
        Subscribes to fixed-wing metrics telemetry with error handling.
        Updates telemetry data and displays it if verbose mode is enabled.
        """
        self.logger.debug("Subscribing to fixed-wing metrics telemetry...")
        try:
            async for metrics in self.drone.telemetry.fixedwing_metrics():
                self.telemetry_data['fixedwing_metrics'] = metrics
                await self.display_telemetry()
        except asyncio.CancelledError:
            self.logger.info("Fixed-wing metrics telemetry subscription cancelled.")
        except Exception as e:
            self.logger.error(f"Error in fixed-wing metrics telemetry subscription: {e}")

    async def subscribe_euler_angle(self) -> None:
        """
        Subscribes to Euler angles telemetry with error handling.
        Updates telemetry data and displays it if verbose mode is enabled.
        """
        self.logger.debug("Subscribing to Euler angles telemetry...")
        try:
            async for euler in self.drone.telemetry.attitude_euler():
                self.telemetry_data['euler_angle'] = euler
                await self.display_telemetry()
        except asyncio.CancelledError:
            self.logger.info("Euler angles telemetry subscription cancelled.")
        except Exception as e:
            self.logger.error(f"Error in Euler angles telemetry subscription: {e}")

    async def subscribe_position_ned(self) -> None:
        """
        Subscribes to Position NED telemetry with error handling.
        Updates telemetry data and displays it if verbose mode is enabled.
        """
        self.logger.debug("Subscribing to Position NED telemetry...")
        try:
            async for position in self.drone.telemetry.position_velocity_ned():
                self.telemetry_data['position_velocity_ned'] = position
                await self.display_telemetry()
        except asyncio.CancelledError:
            self.logger.info("Position NED telemetry subscription cancelled.")
        except Exception as e:
            self.logger.error(f"Error in Position NED telemetry subscription: {e}")

    async def display_telemetry(self) -> None:
        """
        Displays telemetry data in a formatted table using Rich if verbose mode is enabled.
        Clears the console before printing to update the display.
        """
        if not self.verbose:
            return

        table = Table(title="Telemetry Data", show_header=True, header_style="bold magenta")

        # Define table columns
        table.add_column("Metric", style="cyan", no_wrap=True)
        table.add_column("Value", style="magenta")

        # Populate table rows based on available telemetry data
        if 'battery' in self.telemetry_data:
            battery = self.telemetry_data['battery']
            table.add_row("Battery Voltage (V)", f"{battery.voltage_v:.2f}")
            table.add_row("Battery Remaining (%)", f"{battery.remaining_percent:.2f}")

        if 'fixedwing_metrics' in self.telemetry_data:
            metrics = self.telemetry_data['fixedwing_metrics']
            table.add_row("Airspeed (m/s)", f"{metrics.airspeed_m_s:.2f}")
            table.add_row("Throttle (%)", f"{metrics.throttle_percentage:.2f}")
            table.add_row("Climb Rate (m/s)", f"{metrics.climb_rate_m_s:.2f}")

        if 'euler_angle' in self.telemetry_data:
            euler = self.telemetry_data['euler_angle']
            table.add_row("Roll (°)", f"{euler.roll_deg:.2f}")
            table.add_row("Pitch (°)", f"{euler.pitch_deg:.2f}")
            table.add_row("Yaw (°)", f"{euler.yaw_deg:.2f}")
            table.add_row("Timestamp (μs)", f"{euler.timestamp_us}")

        if 'position_velocity_ned' in self.telemetry_data:
            position = self.telemetry_data['position_velocity_ned']
            table.add_row("North (m)", f"{position.position.north_m:.2f}")
            table.add_row("East (m)", f"{position.position.east_m:.2f}")
            table.add_row("Down (m)", f"{position.position.down_m:.2f}")

        # Clear the console before printing the updated table
        self.console.clear()
        self.console.print(table)

    def get_telemetry(self) -> dict:
        """
        Returns a copy of the latest telemetry data.

        :return: Dictionary containing the latest telemetry data.
        """
        return dict(self.telemetry_data)  # Return a shallow copy to prevent external modifications

    async def stop_telemetry(self) -> None:
        """
        Cancels all telemetry subscription tasks with error handling.
        Ensures that all telemetry subscriptions are stopped gracefully.
        """
        self.logger.info("Stopping telemetry subscriptions...")

        # Cancel all subscription tasks
        for task in self.subscriptions:
            task.cancel()

        # Wait for all tasks to be cancelled
        await asyncio.gather(*self.subscriptions, return_exceptions=True)

        # Clear the subscriptions list
        self.subscriptions.clear()

        self.logger.info("All telemetry subscriptions stopped.")
