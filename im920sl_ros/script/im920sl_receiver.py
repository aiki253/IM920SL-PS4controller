#!/usr/bin/env python
import rospy
import serial

def im920sl_receiver():
    rospy.init_node('im920sl_receiver', anonymous=True)
    serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baud_rate', 19200)

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1.0)
        rospy.loginfo("Connected to IM920sL")

        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').rstrip()
                rospy.loginfo("Received data: %s", data)

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
