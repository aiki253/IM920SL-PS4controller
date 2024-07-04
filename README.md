# PS4 to IM920SL Controller

This project allows you to use a PS4 controller to send data via an IM920SL module and integrate it with ROS (Robot Operating System).

## Prerequisites

- ROS (Robot Operating System)
- Python 3.x
- Pygame library
- Pyserial library
- IM920SL module
  - you can buy here (https://www.interplan.co.jp/solution/wireless/im920sl/)
- PS4 controller (or any Pygame-compatible controller)

## Setup

1. Ensure you have ROS and Python 3.x installed on your system.
2. Install the required Python libraries:
   ```
   pip install pygame pyserial
   ```
3. Connect your IM920SL module to your device.
4. Place the `im920sl_controller` package in your ROS workspace.
5. Build your ROS workspace:
   ```
   cd ~/your_ros_workspace
   catkin_make
   ```

## Usage

### Transmitter Side (PS4 Controller to IM920SL)

1. Run the `ps4-to-im920sl-controller.py` script on the device that is set up as the parent device for the IM920SL module:
   ```
   python ps4-to-im920sl-controller.py
   ```

2. Make sure your PS4 controller is connected to your device.

3. The script will now read inputs from your PS4 controller and send them via the IM920SL module.

### Receiver Side (IM920SL to ROS)

1. Launch the ROS node for receiving IM920SL data and publishing it as ROS Joy messages:
   ```
   roslaunch im920sl_controller im920sl_joy.launch
   ```

   You can specify custom serial port and baud rate if needed:
   ```
   roslaunch im920sl_controller im920sl_joy.launch im920sl_serialport:=/dev/ttyUSB0 im920sl_baudrate:=38400
   ```

2. The node will start publishing Joy messages on the `/joy/joy` topic.

3. You can use the published Joy messages in your ROS system as needed.

## Configuration

- The default serial port is set to `/dev/ttyUSB1`. You can change this in the launch file or by passing the `im920sl_serialport` argument.
- The default baud rate is set to 19200. You can change this in the launch file or by passing the `im920sl_baudrate` argument.

## Caution!
- We are not affiliated with interplan in any way.
- This is for personal use only and no responsibility is assumed.

## Notes

- Ensure that the serial port in the script matches the port where your IM920SL module is connected.
- The transmitter script sends data every 0.052 seconds when there's controller input.
- The receiver node will log received data and any errors to the ROS log.
- Check the console output for any error messages or the data being sent/received.

## Troubleshooting

- If no joysticks are found on the transmitter side, ensure your PS4 controller is properly connected and recognized by your system.
- If you encounter serial communication errors, verify that the IM920SL module is correctly connected and that you have the necessary permissions to access the serial port.
- If the ROS node fails to start, check that the specified serial port is correct and not in use by another program.

For more detailed information about the code and its functionality, please refer to the comments within the `ps4-to-im920sl-controller.py` and `im920sl_joy_controller.py` files.
