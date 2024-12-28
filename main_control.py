"""
Professional MAVSDK VTOL Transition Control Script

Arguments:
    --config : (Optional) Path to the configuration YAML file. If not provided, the default is:
               config/transition_parameters_template.yaml
    --yaw    : (Optional) Yaw angle (degrees) to override the `transition_yaw_angle` parameter in the configuration.
               If not specified, the default value is -1 (use initial launch yaw).
"""

import asyncio
import argparse
import yaml
import logging
import sys
from modules.connection_manager import ConnectionManager
from modules.telemetry_handler import TelemetryHandler
from modules.transition_manager import TransitionManager


async def main() -> None:
    """
    Main entry point for the MAVSDK VTOL Transition Control Script.
    Parses arguments, initializes modules, executes the transition, and ensures graceful shutdown.
    """
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="Professional MAVSDK VTOL Transition Control Script"
    )
    parser.add_argument(
        '--config',
        type=str,
        default="config/transition_parameters_template.yaml",  # Default path to the configuration file
        help='Path to transition_parameters.yaml (default: config/transition_parameters_template.yaml)'
    )
    parser.add_argument(
        '--yaw',
        type=float,
        default=-1.0,  # Default yaw angle if not specified (-1 means use initial launch yaw)
        help='Yaw angle (degrees) to override the transition_yaw_angle parameter in the configuration.'
    )
    args = parser.parse_args()

    # Load configuration
    try:
        with open(args.config, 'r') as file:
            config = yaml.safe_load(file)
            if config is None:
                raise ValueError("Configuration file is empty.")
    except FileNotFoundError:
        print(f"Configuration file not found: {args.config}. Exiting.", file=sys.stderr)
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"Error parsing configuration file: {e}. Exiting.", file=sys.stderr)
        sys.exit(1)
    except ValueError as ve:
        print(f"Configuration Error: {ve}. Exiting.", file=sys.stderr)
        sys.exit(1)

    # Override the transition_yaw_angle parameter with the provided yaw argument
    config["transition_yaw_angle"] = args.yaw

    # Set up root logger (configured once)
    logging.basicConfig(
        level=logging.DEBUG if config.get('verbose_mode', False) else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),  # Console output
            logging.FileHandler('mavsdk_vtol_transition.log')  # File output
        ]
    )

    logger = logging.getLogger('MainControl')
    logger.info("Starting MAVSDK VTOL Transition Control Script.")

    # Initialize ConnectionManager
    connection_manager = ConnectionManager(config)
    connection_success = await connection_manager.connect()
    if not connection_success:
        logger.error("Failed to connect to the drone. Exiting.")
        sys.exit(1)

    # Initialize TelemetryHandler with verbose mode
    telemetry_handler = TelemetryHandler(
        drone=connection_manager.drone,
        config=config,
        verbose=config.get('verbose_mode', False)
    )

    # Start telemetry subscriptions in a separate task
    await telemetry_handler.start_telemetry()

    # Initialize TransitionManager
    transition_manager = TransitionManager(
        drone=connection_manager.drone,
        config=config,
        telemetry_handler=telemetry_handler
    )

    # Execute transition logic
    transition_task = asyncio.create_task(transition_manager.execute_transition())

    try:
        # Await the transition task to complete and get the result
        transition_result = await transition_task

        if transition_result == "success":
            logger.info("Transition completed successfully.")
        elif transition_result == "failure":
            logger.error("Transition failed.")
        else:
            logger.warning(f"Transition completed with unknown status: {transition_result}")

    except asyncio.CancelledError:
        logger.info("Main tasks cancelled.")
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received. Cancelling tasks...")
        transition_task.cancel()
        # Optionally, you can cancel telemetry tasks here if they are long-running
        await transition_task
    except Exception as e:
        logger.error(f"An unexpected error occurred during transition: {e}")
    finally:
        # Ensure that telemetry subscriptions are stopped
        await telemetry_handler.stop_telemetry()

        # Disconnect from the drone
        await connection_manager.disconnect()

        logger.info("Shutdown complete.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        # Catch any exception that wasn't handled in main
        logging.getLogger('MainControl').error(f"An unexpected error occurred: {e}")
        sys.exit(1)
