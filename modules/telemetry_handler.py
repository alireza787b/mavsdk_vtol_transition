import asyncio
import logging
from mavsdk import System
from mavsdk.telemetry import Battery, FixedwingMetrics, EulerAngle, PositionNed
from rich.console import Console
from rich.table import Table


class TelemetryHandler:
    """
    Handles telemetry data retrieval and logging.
    """

    def __init__(self, drone: System, config: dict, verbose: bool = False):
        self.drone = drone
        self.update_interval = config.get('telemetry_update_interval', 1.0)
        self.verbose = verbose
        self.logger = logging.getLogger('TelemetryHandler')
        self.telemetry_data = {}
        self.console = Console()

    async def start_telemetry(self):
        """ Initializes telemetry subscriptions based on verbosity. """
        self.logger.info("Starting telemetry subscriptions...")
        await asyncio.gather(
            self.subscribe_battery(),
            self.subscribe_fixedwing_metrics(),
            self.subscribe_euler_angle(),
            self.subscribe_position_ned())

    async def subscribe_battery(self):
        """ Subscribes to battery telemetry with error handling. """
        try:
            async for battery in self.drone.telemetry.battery():
                self.telemetry_data['battery'] = battery
                await self.display_telemetry()
        except Exception as e:
            self.logger.error(f"Error in battery subscription: {e}")

    async def subscribe_fixedwing_metrics(self):
        """ Subscribes to fixedwing metrics telemetry with error handling. """
        try:
            async for metrics in self.drone.telemetry.fixedwing_metrics():
                self.telemetry_data['fixedwing_metrics'] = metrics
                await self.display_telemetry()
        except Exception as e:
            self.logger.error(f"Error in fixedwing metrics subscription: {e}")

    async def subscribe_euler_angle(self):
        """ Subscribes to Euler angles telemetry with error handling. """
        try:
            async for euler in self.drone.telemetry.attitude_euler():
                self.telemetry_data['euler_angle'] = euler
                await self.display_telemetry()
        except Exception as e:
            self.logger.error(f"Error in Euler angle subscription: {e}")

    async def subscribe_position_ned(self):
        """ Subscribes to Position NED telemetry with error handling. """
        try:
            async for position in self.drone.telemetry.position_velocity_ned():
                self.telemetry_data['position_velocity_ned'] = position
                await self.display_telemetry()
        except Exception as e:
            self.logger.error(f"Error in position NED subscription: {e}")


    async def display_telemetry(self):
        """ Displays telemetry data in a formatted table using Rich. """
        if not self.verbose:
            return
        table = Table(title="Telemetry Data")

        # Define table columns
        table.add_column("Metric", style="cyan")
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


        # Print the updated table using Rich's console print method
        await asyncio.sleep(0)  # Yield control back to the event loop
        self.console.print(table)

    def get_telemetry(self):
        """ Returns the latest telemetry data. """
        return dict(self.telemetry_data)  # Return a copy of the data

    async def stop_telemetry(self):
        """ Cancels all telemetry subscription tasks with error handling. """
        # Implementation remains unchanged from previous versions.
        self.logger.info("Stopping telemetry subscriptions...")
        
        for task in self.subscriptions:
            task.cancel()

        await asyncio.gather(*self.subscriptions, return_exceptions=True)
        
        # Log completion of stopping tasks
        self.logger.info("All telemetry subscriptions stopped.")
