## Overview

The **mavsdk_vtol_transition** module enables custom transition modes for tailsitter VTOL drones using MAVSDK and PX4. This guide provides step-by-step instructions to set up, configure, and execute transition modes.

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/mavsdk_vtol_transition.git
cd mavsdk_vtol_transition
```

### 2. Setup Virtual Environment

```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Transition Parameters

1. **Copy the Template Configuration:**

    ```bash
    cp templates/transition_parameters_template.yaml config/drone_specific/drone1_transition_parameters.yaml
    ```

2. **Customize Parameters:**

    - Open `config/drone_specific/drone1_transition_parameters.yaml` in your preferred editor.
    - Adjust parameters as needed to suit your drone's specifications and desired transition behavior.

### 5. Add Transition Modes

Each transition mode resides in its own module within the `modules/` directory.

- **Adding a New Transition Mode:**

    1. **Create Module Directory:**

        ```bash
        mkdir modules/new_transition_mode
        mkdir modules/new_transition_mode/config
        ```

    2. **Create Essential Files:**

        ```bash
        touch modules/new_transition_mode/__init__.py
        touch modules/new_transition_mode/transition_control.py
        cp templates/transition_parameters_template.yaml modules/new_transition_mode/config/transition_parameters.yaml
        ```

    3. **Implement Transition Logic:**

        - Develop the `transition_control.py` script based on the specific requirements of the new transition mode.

    4. **Update `main_control.py`:**

        - Modify `scripts/main_control.py` to include the new transition mode.

### 6. Execute Transition Mode

- **Run the Transition Script:**

    ```bash
    python scripts/main_control.py --mode pitch_climb --config config/drone_specific/drone1_transition_parameters.yaml
    ```

    - **Parameters:**
        - `--mode`: Specifies the transition mode to execute (e.g., `pitch_climb`, `dive_accelerate`).
        - `--config`: Path to the configuration YAML file.

### 7. Running in Simulation

1. **Launch PX4 with Gazebo:**

    ```bash
    make px4_sitl gazebo_tailsitter
    ```

2. **Execute Transition Script:**

    ```bash
    python scripts/main_control.py --mode pitch_climb --config config/drone_specific/drone1_transition_parameters.yaml
    ```

### 8. Monitoring and Logging

- **Telemetry Monitoring:**
    - The transition scripts print real-time telemetry data (altitude, airspeed, pitch, throttle) to the console.
  
- **Logging:**
    - Implement logging mechanisms within each transition module to record detailed telemetry data for post-flight analysis.

### 9. Testing

- **Unit Tests:**

    ```bash
    python -m unittest discover tests/unit_tests/
    ```

- **Integration Tests:**

    ```bash
    python -m unittest discover tests/integration_tests/
    ```

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## Troubleshooting

- **Offboard Mode Fails to Start:**
    - Ensure the drone is properly connected and armed.
    - Verify that no other offboard controllers are active.
    - Check for any errors in the console output for clues.

- **Transition Timeout Exceeded:**
    - Review configuration parameters to ensure they are set appropriately.
    - Check drone's sensor health and calibration.

## Contact

For questions or feedback, please open an issue or LinkedIn [https://www.linkedin.com/in/alireza787b/](https://www.linkedin.com/in/alireza787b/).
