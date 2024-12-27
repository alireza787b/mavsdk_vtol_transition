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
    parser.add_argument('--config', type=str, required=True, help='Path to transition_parameters.yaml')
    args = parser.parse_args()

    # Load configuration
    with open(args.config, 'r') as file:
        config = yaml.safe_load(file)

    # Set up root logger
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    )

    # Initialize ConnectionManager
    connection_manager = ConnectionManager(config)
    await connection_manager.connect()

    # Initialize TelemetryHandler
    telemetry_handler = TelemetryHandler(connection_manager.drone, config)

    # Start telemetry in a separate task
    telemetry_task = asyncio.create_task(telemetry_handler.start_telemetry())

    # Initialize VTOLTransition
    vtol_transition = VTOLTransition(connection_manager.drone, config, telemetry_handler)

    # Arm and takeoff
    await vtol_transition.arm_and_takeoff()

    # Start offboard mode
    await vtol_transition.start_offboard()

    # Execute transition logic in a separate task
    transition_task = asyncio.create_task(vtol_transition.execute_transition())

    try:
        # Keep the main thread alive while tasks are running
        await asyncio.gather(telemetry_task, transition_task)
    except asyncio.CancelledError:
        logging.info("Main tasks cancelled.")
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received. Cancelling tasks...")
        transition_task.cancel()
        telemetry_task.cancel()
        await asyncio.gather(telemetry_task, transition_task, return_exceptions=True)
    finally:
        logging.info("Shutdown complete.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
