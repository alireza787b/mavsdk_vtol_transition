# scripts/main_control.py

import asyncio
import argparse
import yaml
import logging
from modules.connection_manager import ConnectionManager
from modules.telemetry_handler import TelemetryHandler
from modules.transition_logic import VTOLTransition


async def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Professional MAVSDK VTOL Transition Control Script")
    parser.add_argument(
        '--config',
        type=str,
        default='config/transition_parameters_template.yaml',
        help='Path to transition_parameters.yaml (default: config/transition_parameters_template.yaml)'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose telemetry logging'
    )
    args = parser.parse_args()

    # Set up root logger first to capture any errors during configuration loading
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    )
    logger = logging.getLogger('MainControl')

    # Load configuration
    try:
        with open(args.config, 'r') as file:
            config = yaml.safe_load(file)
    except FileNotFoundError:
        logger.error(f"Configuration file not found: {args.config}")
        return
    except yaml.YAMLError as e:
        logger.error(f"Error parsing configuration file: {e}")
        return

    # Update logging level based on configuration or command-line arguments
    verbose_mode = config.get('verbose_mode', False) or args.verbose
    if verbose_mode:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    # Initialize ConnectionManager
    connection_manager = ConnectionManager(config)
    await connection_manager.connect()

    # Initialize TelemetryHandler with verbose mode
    telemetry_handler = TelemetryHandler(
        connection_manager.drone,
        config,
        verbose=verbose_mode
    )

    # Start telemetry subscriptions
    await telemetry_handler.start_telemetry()

    # Initialize VTOLTransition
    vtol_transition = VTOLTransition(connection_manager.drone, config, telemetry_handler)

    # Arm and takeoff (if enabled)
    await vtol_transition.arm_and_takeoff()

    # Start offboard mode
    await vtol_transition.start_offboard()

    # Execute transition logic in a separate task (currently disabled)
    transition_task = asyncio.create_task(vtol_transition.execute_transition())

    try:
        # Keep the main thread alive while telemetry and transition tasks are running
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received. Cancelling tasks...")
        transition_task.cancel()
        await transition_task
    finally:
        # Stop telemetry subscriptions
        await telemetry_handler.stop_telemetry()
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        logging.getLogger('MainControl').error(f"An unexpected error occurred: {e}")