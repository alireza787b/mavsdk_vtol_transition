"""
Professional MAVSDK VTOL Transition Control Script

Arguments:
    --config : (Optional) Path to the configuration YAML file. If not provided, the default is:
               config/transition_parameters_template.yaml
    --yaw    : (Optional) Yaw angle (degrees) to override the `transition_yaw_angle` parameter in the configuration.
               If not specified, the default value is 0.0 degrees.
"""

import asyncio
import argparse
import yaml
import logging
from modules.connection_manager import ConnectionManager
from modules.telemetry_handler import TelemetryHandler
from modules.transition_manager import TransitionManager  # Use TransitionManager instead of direct logic

async def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Professional MAVSDK VTOL Transition Control Script")
    parser.add_argument(
        '--config',
        type=str,
        default="config/transition_parameters_template.yaml",  # Default path to the configuration file
        help='Path to transition_parameters.yaml (default: config/transition_parameters_template.yaml)'
    )
    parser.add_argument(
        '--yaw',
        type=float,
        default=0.0,  # Default yaw angle if not specified
        help='Yaw angle (degrees) to override the transition_yaw_angle parameter in the configuration.'
    )
    args = parser.parse_args()

    # Load configuration
    try:
        with open(args.config, 'r') as file:
            config = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Configuration file not found: {args.config}. Exiting.")
        return
    except yaml.YAMLError as e:
        print(f"Error parsing configuration file: {e}. Exiting.")
        return

    # Override the transition_yaw_angle parameter with the provided yaw argument
    config["transition_yaw_angle"] = args.yaw

    # Set up root logger with both console and file handlers
    logger = logging.getLogger('MainControl')
    logger.setLevel(logging.DEBUG if config.get('verbose_mode', False) else logging.INFO)

    # Console handler for real-time telemetry display and logging
    console_handler = logging.StreamHandler()
    console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)

    # File handler for persistent logging
    file_handler = logging.FileHandler('mavsdk_vtol_transition.log')
    file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(file_formatter)
    logger.addHandler(file_handler)

    logger.info("Starting MAVSDK VTOL Transition Control Script.")

    # Initialize ConnectionManager
    connection_manager = ConnectionManager(config)
    await connection_manager.connect()

    # Initialize TelemetryHandler with verbose mode
    telemetry_handler = TelemetryHandler(
        connection_manager.drone,
        config,
        verbose=config.get('verbose_mode', False)
    )

    # Start telemetry subscriptions in a separate task
    telemetry_task = asyncio.create_task(telemetry_handler.start_telemetry())

    # Initialize TransitionManager
    transition_manager = TransitionManager(
        connection_manager.drone,
        config,
        telemetry_handler
    )

    # Execute transition logic in a separate task
    transition_task = asyncio.create_task(transition_manager.execute_transition())

    try:
        # Keep the main thread alive while telemetry and transition tasks are running
        await asyncio.gather(telemetry_task, transition_task)
    except asyncio.CancelledError:
        logger.info("Main tasks cancelled.")
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received. Cancelling tasks...")
        transition_task.cancel()
        telemetry_task.cancel()
        await asyncio.gather(telemetry_task, transition_task, return_exceptions=True)
    finally:
        # Stop telemetry subscriptions
        await telemetry_handler.stop_telemetry()
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        logging.getLogger('MainControl').error(f"An unexpected error occurred: {e}")
