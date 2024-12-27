# scripts/validate_config.py

import yaml
import sys

REQUIRED_FIELDS = {
    "connection_type": str,
    "connection_endpoint": str,
    "telemetry_update_interval": float,
    "TRANSITION_SAFE_ALTITUDE": float,
    "enable_takeoff": bool,
    "verbose_mode": bool
}

def validate_config(config_path):
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Configuration file not found: {config_path}")
        return False
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return False

    missing_fields = []
    incorrect_types = []

    for field, field_type in REQUIRED_FIELDS.items():
        if field not in config:
            missing_fields.append(field)
        elif not isinstance(config[field], field_type):
            incorrect_types.append((field, field_type.__name__, type(config[field]).__name__))

    if missing_fields:
        print(f"Missing required fields: {', '.join(missing_fields)}")
    if incorrect_types:
        for field, expected, actual in incorrect_types:
            print(f"Incorrect type for '{field}': Expected {expected}, got {actual}")

    if missing_fields or incorrect_types:
        return False

    print("Configuration file is valid.")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python validate_config.py <config_file.yaml>")
        sys.exit(1)
    config_file = sys.argv[1]
    if not validate_config(config_file):
        sys.exit(1)
# scripts/validate_config.py

import yaml
import sys

REQUIRED_FIELDS = {
    "connection_type": str,
    "connection_endpoint": str,
    "telemetry_update_interval": float,
    "TRANSITION_SAFE_ALTITUDE": float,
    "enable_takeoff": bool,
    "verbose_mode": bool
}

def validate_config(config_path):
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Configuration file not found: {config_path}")
        return False
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return False

    missing_fields = []
    incorrect_types = []

    for field, field_type in REQUIRED_FIELDS.items():
        if field not in config:
            missing_fields.append(field)
        elif not isinstance(config[field], field_type):
            incorrect_types.append((field, field_type.__name__, type(config[field]).__name__))

    if missing_fields:
        print(f"Missing required fields: {', '.join(missing_fields)}")
    if incorrect_types:
        for field, expected, actual in incorrect_types:
            print(f"Incorrect type for '{field}': Expected {expected}, got {actual}")

    if missing_fields or incorrect_types:
        return False

    print("Configuration file is valid.")
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python validate_config.py <config_file.yaml>")
        sys.exit(1)
    config_file = sys.argv[1]
    if not validate_config(config_file):
        sys.exit(1)
