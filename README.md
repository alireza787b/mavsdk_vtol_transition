# MAVSDK VTOL Transition

![License](https://img.shields.io/badge/license-apache-blue.svg)
![GitHub Stars](https://img.shields.io/github/stars/alireza787b/mavsdk_vtol_transition.svg)
![GitHub Forks](https://img.shields.io/github/forks/alireza787b/mavsdk_vtol_transition.svg)
![GitHub Issues](https://img.shields.io/github/issues/alireza787b/mavsdk_vtol_transition.svg)

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
  - [Configuration Parameters](#configuration-parameters)
- [Usage](#usage)
  - [Running the Program](#running-the-program)
  - [Connection Options](#connection-options)
- [Command-Line Arguments](#command-line-arguments)
- [Safety Notice](#safety-notice)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Overview

The **MAVSDK VTOL Trasistion** is a professional-grade transition logic designed for  Vertical Take-Off and Landing (VTOL) drones, specifally for tailsitters currently. It manages essential phases such as arming, takeoff, initial climb, throttle and tilt ramping, and seamless transition to fixed-wing mode. Enhanced with over-tilting capabilities and comprehensive failsafe mechanisms, this program ensures safe, efficient, and reliable drone operations tailored to various use cases.

> **Disclaimer:** This software is not yet tested in real-world scenarios. Use it at your own risk and responsibility.

## Features

- **Dual Transition Modes:** Supports both climb-then-tilt (dive) and continuous climb-and-tilt transitions.
- **Over-Tilting Capability:** Enables the drone to tilt beyond the initial maximum pitch to gain additional airspeed when necessary.
- **Comprehensive Failsafes:** Monitors various telemetry parameters to ensure safe operations and aborts transitions if safety thresholds are breached.
- **Configurable Parameters:** Easily customizable through YAML configuration files to suit different operational requirements.
- **Flexible Connection Options:** Supports both UDP and serial connections for MAVLink communication.
- **Detailed Logging:** Provides verbose telemetry logging for in-depth monitoring and debugging.

## Prerequisites

- **Operating System:** Windows, Linux, or macOS
- **Python:** Version 3.7 or higher
- **MAVSDK:** Installed separately (see [Installation](#installation))
- **Virtual Environment:** Recommended for dependency management

## Installation

1. **Clone the Repository:**

   ```bash
   cd ~
   git clone https://github.com/alireza787b/mavsdk_vtol_transition.git
   cd mavsdk_vtol_transition
   ```

2. **Create a Virtual Environment:**

   ```bash
   python -m venv venv
   ```

3. **Activate the Virtual Environment:**

   - **Windows:**

     ```bash
     venv\Scripts\activate
     ```

   - **Linux/macOS:**

     ```bash
     source venv/bin/activate
     ```

4. **Install Dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

5. **Install MAVSDK Server Binary:**

   Depends on your operating system you might need to download the appropriate MAVSDK Server binary for your operating system from the [MAVSDK Releases](https://github.com/mavlink/MAVSDK/releases/) page and follow the installation instructions.

## Configuration

All operational parameters are defined in YAML configuration files located in the `config` directory. You can customize these parameters to fit your specific use case.

### Configuration Parameters

| Parameter                        | Type   | Description                                                                                                             |
|----------------------------------|--------|-------------------------------------------------------------------------------------------------------------------------|
| `transition_type`                | string | Type of transition logic to use. Options: `"tailsitter_pitch_program"`, `"other_transition_type"`                       |
| `enable_takeoff`                 | bool   | Enable or disable the takeoff functionality.                                                                            |
| `safety_lock`                    | bool   | Acts as a safety switch.                                                                                                 |
| `verbose_mode`                   | bool   | Enable detailed telemetry logging.                                                                                      |
| `connection_type`                | string | Type of connection for MAVLink. Options: `"udp"`, `"serial"`.                                                          |
| `connection_endpoint`            | string | Endpoint for the connection. Example for UDP: `udp://:14540`. Example for Serial: `serial:///dev/ttyUSB0:57600`             |
| `cycle_interval`                 | float  | Interval in seconds to update telemetry data and send offboard commands.                                                |
| `initial_takeoff_height`         | float  | Target altitude (meters) for the initial auto takeoff phase.                                                           |
| `initial_climb_rate`             | float  | Climb rate (m/s) during the initial climb phase.                                                                       |
| `initial_climb_height`           | float  | Target altitude (meters) for the initial climb phase.                                                                  |
| `secondary_climb_rate`           | float  | Climb rate (m/s) during the secondary climb phase.                                                                     |
| `transition_base_altitude`       | float  | Altitude (meters) at which the transition starts.                                                                       |
| `transition_yaw_angle`           | float  | Yaw angle (degrees) to maintain during the transition.                                                                 |
| `throttle_ramp_time`             | float  | Time (seconds) to ramp throttle to maximum.                                                                             |
| `max_throttle`                    | float  | Maximum throttle as a fraction of full thrust (0.0 to 1.0).                                                             |
| `max_tilt_pitch`                 | float  | Maximum pitch angle (degrees) for tilt. Negative values indicate downward tilt.                                         |
| `forward_transition_time`         | float  | Time (seconds) to transition to maximum tilt.                                                                           |
| `over_tilt_enabled`              | bool   | Enable over-tilting to gain additional airspeed.                                                                         |
| `max_allowed_tilt`               | float  | Maximum allowable tilt (degrees) during over-tilting. Negative for downward tilt.                                       |
| `transition_air_speed`           | float  | Airspeed (m/s) to trigger fixed-wing mode.                                                                                |
| `acceleration_factor`           | float  | Temporary velocity setpoint multiplier after successful transition.                                                                                |
| `altitude_failsafe_threshold`    | float  | Altitude (meters) below which to abort the transition.                                                                   |
| `climb_rate_failsafe_threshold`  | float  | Climb rate (m/s) below which to abort the transition. Set to negative if diving/over-tilt is used.                        |
| `altitude_loss_limit`            | float  | Maximum allowable altitude loss (meters) during over-tilting.                                                             |
| `max_pitch_failsafe`             | float  | Maximum pitch angle (degrees) before aborting the transition.                                                            |
| `max_roll_failsafe`              | float  | Maximum roll angle (degrees) before aborting the transition.                                                             |
| `max_altitude_failsafe`          | float  | Maximum altitude (meters) before aborting the transition.                                                                |
| `return_to_launch_on_abort`      | bool   | Whether to return to home after aborting the transition.                                                                 |
| `failsafe_multicopter_transition` | bool   | Whether to transition to multi-copter mode as part of abort procedures.                                                   |
| `transition_timeout`             | float  | Time (seconds) before aborting the transition.                                                                            |
| `post_transition_action`         | string  | Action to perform after successful transition. Options:  `"return_to_launch"`, `"start_mission"`, `"hold"`, `"continue_current_heading"`  |

All parameters can be found in the `config` folder. Users can create custom configuration files based on the provided template to suit their specific requirements.

## Usage
### Running MAVSDK_SERVER
Depends on your operating system, you might need to run the mavsdk_server binary (download in last steps) manually. mavsdk_server for windows and generic musl Linux (Ubuntu,etc. )is already included in the repository. If you are using Windows or Ubuntu, you can type this command to run that. Feel free to adjust based on the correct file name that you have downlaoded for your opeating system.
- **For Windows:**
  ```bash
  .\mavsdk_server_win32.exe
  ```
- **For Ubuntu:**
  ```bash
  ./mavsdk_server_musl_x86_64
  ```
  For other operating systems, download currect binary from  [MAVSDK Releases](https://github.com/mavlink/MAVSDK/releases/) page, and use the currect file name.

### Running the Program

To execute the transition program, use the `main_control.py` script with the desired configuration file, in a new terminal. The program supports both Windows and Linux environments.

  ```bash
  cd ~/mavsdk_vtol_transition
  ```

- **Climb then Tilt (Dive) Transition:**


  ```bash
  python main_control.py --config ./config/dive_pitch_program.yaml
  ```

- **Continuous Climb and Tilt Transition:**

  ```bash
  python main_control.py --config ./config/climb_pitch_program.yaml
  ```

Users can customize the parameters in these configuration files to fit their specific use cases.
Make sure your virtual environment is active.

### Connection Options

The program supports two primary connection types for MAVLink communication: **UDP** and **Serial**. Configuration for these connections is handled entirely through the YAML configuration files. Additionally, specific setups like running SITL on Windows Subsystem for Linux (WSL) or using a companion computer may require additional configurations.


#### 1. Serial Connection

- **Configuration:**

  ```yaml
  connection_type: "serial"
  connection_endpoint: "serial:///dev/ttyS0:57600"
  ```

  Replace `/dev/ttyS0` with the appropriate serial port on your system and `57600` with the correct baud rate.

- **Usage:**

  When running on a companion computer like a Raspberry Pi, you can use the serial interface for MAVLink communication. Depending on your setup, you might need to use a MAVLink router to bridge serial and UDP connections.

  **Example with MAVLink Router:**

  ```bash
  mavlink-routerd -e 172.21.144.1:14550 -e 172.21.144.1:14540 /dev/ttyS0:57600
  ```

  Refer to the [Mavlink Anywhere Tutorial](https://www.youtube.com/watch?v=_QEWpoy6HSo) for detailed instructions.

#### 2. WSL SITL on Windows

- **Configuration Example:**

  ```yaml
  connection_type: "udp"
  connection_endpoint: "udp://127.0.0.1:14540"
  ```

- **Usage:**

  If running SITL on Windows Subsystem for Linux (WSL) and executing the code on Windows, use tools like `mavlink-router` to route MAVLink streams between WSL and Windows:

  ```bash
  mavlink-routerd -e 172.21.144.1:14550 -e 172.21.144.1:14540 127.0.0.1:14550
  ```

  Refer to the [Mavlink Anywhere Guide](https://www.youtube.com/watch?v=_QEWpoy6HSo) for more details.

#### 3. SITL on Linux

- **Configuration Example:**

  ```yaml
  connection_type: "udp"
  connection_endpoint: "udp://:14540"
  ```

- **Usage:**

  When running SITL entirely on Linux, the default MAVLink streams on ports `14540` and `14550` are available. Ensure that your configuration points to the correct UDP endpoints without needing additional routing.

## Command-Line Arguments

- `--config`: **(Optional)** Path to the YAML configuration file. If not specified will use the default template configuration.
- `--yaw`: **(Optional)** Transition heading angle in degrees.

**Example:**

```bash
python main_control.py --config ./config/dive_pitch_program.yaml --yaw 90
```

This command runs the transition using the `dive_pitch_program.yaml` configuration and sets the transition yaw angle to `90` degrees.
Depends on configured pitch program, you might need to adjust the PX4 pitch failure detection failsafe.

## Safety Notice

**Important:** This software has not been tested in real-world conditions. Use it at your own risk. Ensure you understand the implications of the configuration parameters and thoroughly test in a controlled environment before deploying on actual hardware.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any enhancements or bug fixes. Ensure that your code follows best practices and includes appropriate documentation.

## License

This project is licensed under the Apache License. See the [LICENSE](LICENSE) file for details.

## Contact

For questions or feedback, please open an [issue](https://github.com/alireza787b/mavsdk_vtol_transition/issues) or connect with me on [LinkedIn](https://www.linkedin.com/in/alireza787b/).

