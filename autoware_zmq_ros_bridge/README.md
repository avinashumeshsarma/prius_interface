# Autoware ZMQ ROS Bridge

This node bridges between Autoware and OpenPilot using ZMQ and Cap'n Proto for communication.

## Features

- **ZMQ Communication**: Receives car state from OpenPilot and sends control commands back
- **ROS Integration**: Publishes vehicle status messages and receives control commands
- **Manual Override Detection**: Automatically detects when driver takes manual control
- **Auto-Reenable**: Automatically re-enables OpenPilot after a configurable timeout

## Parameters

- `capnp_dir`: Directory containing Cap'n Proto schema files
- `publish_rate_hz`: Rate at which ROS messages are published (default: 10.0 Hz)
- `openpilot_auto_reenable_timeout`: Timeout in seconds before auto-reenabling OpenPilot after manual override (default: 5.0 seconds)

## Auto-Reenable Feature

The node automatically detects manual override conditions (brake pressed, gas pressed) and disables OpenPilot. After a configurable timeout period, it automatically re-enables OpenPilot.

### How it works:

1. **Manual Override Detection**: When brake or gas is pressed while cruise control is enabled, OpenPilot is disabled
2. **Timer Start**: A countdown timer starts for the specified timeout duration
3. **Auto-Reenable**: After the timeout expires, OpenPilot is automatically re-enabled
4. **Early Cancellation**: If manual override ends before timeout, the timer is cancelled

### Configuration:


## Usage

1. Ensure OpenPilot is running and sending car state messages on port 9041
2. Launch the node with appropriate parameters
3. The node will automatically handle OpenPilot state management

## Topics

### Subscribed Topics
- `/control/command/control_cmd`: Control commands from Autoware
- `/control/command/emergency_cmd`: Emergency commands
- `/control/command/gear_cmd`: Gear commands
- `/control/command/hazard_lights_cmd`: Hazard light commands
- `/control/command/turn_indicators_cmd`: Turn signal commands

### Published Topics
- `/vehicle1/status/control_mode`: Vehicle control mode status
- `/vehicle1/status/gear_status`: Gear status
- `/vehicle1/status/hazard_lights_status`: Hazard lights status
- `/vehicle1/status/steering_status`: Steering status
- `/vehicle1/status/turn_indicators_status`: Turn indicators status
- `/vehicle1/status/velocity_status`: Velocity status

### Services
- `/openpilot/enable`: Service to manually enable/disable OpenPilot

## ZMQ Ports

- **Subscriber**: `tcp://127.0.0.1:9041` (receives car state from OpenPilot)
- **Publisher**: `tcp://127.0.0.1:63225` (sends control commands to OpenPilot)
