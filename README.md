# IM920SL-PS4controller

This project allows you to use a PS4 controller to send data via an IM920SL module.

## Prerequisites

- Python 3.9
- Pygame library
- Pyserial library
- IM920SL module
- PS4 controller (or any Pygame-compatible controller)
- ROS-noetic

## Setup

1. Ensure you have Python 3.9 installed on your system.
2. Install the required Python libraries:
   ```
   pip install pygame pyserial
   ```
3. Connect your IM920SL module to your device.

## Usage

1. Run the `ps4-to-im920sl-controller.py` script on the device that is set up as the parent device for the IM920SL module:
   ```
   python ps4-to-im920sl-controller.py
   ```

2. Place the `im920sl_ros` package in your ROS workspace.

3. Make sure your PS4 controller is connected to your device.

4. The script will now read inputs from your PS4 controller and send them via the IM920SL module.

## Notes

- Ensure that the serial port in the script (`/dev/ttyUSB0`) matches the port where your IM920SL module is connected.
- The script sends data every 0.052 seconds when there's controller input.
- Check the console output for any error messages or the data being sent.

## Troubleshooting

- If no joysticks are found, ensure your PS4 controller is properly connected and recognized by your system.
- If you encounter serial communication errors, verify that the IM920SL module is correctly connected and that you have the necessary permissions to access the serial port.

For more detailed information about the code and its functionality, please refer to the comments within the `ps4-to-im920sl-controller.py` file.
