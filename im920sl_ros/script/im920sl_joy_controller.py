#!/usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import Joy

def round_to_nearest(value):
    if abs(value - 1.0) <= 0.03:
        return 1.0
    elif abs(value - 0.0) <= 0.03:
        return 0.0
    elif abs(value - (-1.0)) <= 0.03:
        return -1.0
    else:
        return value

def decode_controller_data(data):
    # Decode buttons
    buttons_hex = data[:4]
    buttons_int = int(buttons_hex, 16)
    buttons = [bool(buttons_int & (1 << i)) for i in range(13)]

    # Decode axes
    axes_hex = data[4:16]
    axes = [int(axes_hex[i:i+2], 16) for i in range(0, 12, 2)]
    axes = [-(val - 128) / 128.0 for val in axes]
    axes = [round_to_nearest(val) for val in axes]

    # Decode hats
    hats_hex = data[16:20]
    hat_x = int(hats_hex[:2], 16)
    hat_y = int(hats_hex[2:], 16)
    hat_x = (hat_x - 128) / 128.0
    hat_y = (hat_y - 128) / 128.0
    hat_x = round_to_nearest(hat_x)
    hat_y = round_to_nearest(hat_y)
    hats = [hat_x, hat_y]

    return buttons, axes, hats

def im920sl_receiver():
    rospy.init_node('im920sl_receiver', anonymous=True)
    serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
    baud_rate = rospy.get_param('~baud_rate', 19200)
    joy_pub = rospy.Publisher('joy', Joy, queue_size=10)

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1.0)
        rospy.loginfo("Connected to IM920sL")

        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').rstrip()
                rospy.loginfo("Received data: %s", data)

                if ':' in data:
                    controller_data = data.split(':')[-1].replace(',', '')
                    if len(controller_data) == 20:
                        buttons, axes, hats = decode_controller_data(controller_data)
                        joy_msg = Joy()
                        joy_msg.header.stamp = rospy.Time.now()
                        joy_msg.buttons = buttons
                        joy_msg.axes = axes + hats
                        joy_pub.publish(joy_msg)
                    else:
                        rospy.logwarn("Received invalid controller data: %s", controller_data)
                else:
                    rospy.logwarn("Received data without colon: %s", data)

    except serial.SerialException:
        rospy.logerr("Failed to connect to IM920sL")

    finally:
        ser.close()
        rospy.loginfo("Disconnected from IM920sL")

if __name__ == '__main__':
    try:
        im920sl_receiver()
    except rospy.ROSInterruptException:
        pass