#!/usr/bin/env python3

# Respect to ancestors: alpogant & berkay & chatgpt

# Yaw angle and x velocity are sending to embedded system via serial
# Wheel angular velocities ang gnss datas are receiving from embedded system via serial
# Handles the LED operation

import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
import serial
from binascii import unhexlify
import struct


class ControlVCU:
    def __init__(self):
        rospy.init_node("vcu_serial_node", anonymous=False)

        self.angular_z = None
        self.linear_x = None
        self.angular_dir = None
        self.linear_dir = None

        self.status = "AUTO"
        self.led_color = "2"

        self.serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0.02)
        self.send_msg = ""
        self.recv_msg = ""
        self.wheel_speeds = [0,0,0,0]
        self.gnss = [0,0,0,0,0] # lat, long, yaw, cog, sog

        self.rate = rospy.Rate(10)
        self.init()
        self.run()

    def init(self):
        rospy.loginfo("VCU communucation is established")
        rospy.Subscriber("/drive_system/twist", Twist, self.twist_cb)
        rospy.Subscriber("/drive_system/status", String, self.status_cb)
        self.rpm_publisher = rospy.Publisher("/drive_system/wheel_angular_velocities", Float64MultiArray, queue_size=10)
        self.pub_gnss = rospy.Publisher("/gnss_data", Float64MultiArray, queue_size=10)
        self.vcu_publisher = rospy.Publisher("/drive_system/vcu_data", String, queue_size=10)

    def run(self):
        rospy.loginfo("OPERATION Started")
        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.angular_z is not None:
                self.send_msg = ""  # Clear send_msg before appending new message
                self.send_msg += "S"
                self.send_msg += str(self.angular_dir) # Yaw direction (0 or 1)
                self.send_msg += str(abs(int(float("{:.2f}".format(self.angular_z)) * 100))).zfill(3) # Yaw
                self.send_msg += str(self.linear_dir) # X direction (0 or 1)
                self.send_msg += str(abs(int(float("{:.3f}".format(self.linear_x)) * 1000))).zfill(4) # X
                self.send_msg += self.led_color  # Red "1" (AUTO), Blue "2" (TELEOP), Flashing Green "3" (SUCCESS), No led "4"
                self.send_msg += "F"
                # print(self.send_msg)
                self.serial.write(self.send_msg.encode())
                self.get_feedback()


    def get_feedback(self):
        raw_data = 0
        raw_data = self.serial.read(1)
        if raw_data and raw_data[0] == 83:
            raw_data = raw_data + self.serial.read(57)
        #rospy.loginfo(raw_data[])
        try:
            if raw_data[0] == 83 and raw_data[57] == 88:    
                self.wheel_speeds[0] = raw_data[1:9]

                rospy.loginfo(f"rawdata: {raw_data}")
                self.wheel_speeds[0] = int(raw_data[1:9].decode())
                self.wheel_speeds[1] = int(raw_data[9:17].decode()) #struct.unpack(">i", bytes.fromhex(self.recv_msg[9:17]))[0] #RIGHT REAR
                self.wheel_speeds[2] = int(raw_data[17:25].decode())# struct.unpack(">i", bytes.fromhex(self.recv_msg[17:25]))[0] #LEFT FRONT
                self.wheel_speeds[3] = int(raw_data[25:33].decode()) # struct.unpack(">i", bytes.fromhex(self.recv_msg[25:33]))[0] #LEFT REAR
                self.gnss[0] = struct.unpack("<i", raw_data[41:45])[0]* (1e-7)  #lat
                self.gnss[1] = struct.unpack("<i", raw_data[45:49])[0]* (1e-7)  #long
                self.gnss[2] = 0 #-int.from_bytes(data[21:23],byteorder='little') * (1e-4) #yaw
                self.gnss[3] = struct.unpack("<i", raw_data[49:53])[0]* (1e-5) #cog 
                self.gnss[4] = struct.unpack("<i", raw_data[53:57])[0]* (1e-3) #sog
                rospy.loginfo(f"wheel data: {self.wheel_speeds}")
                rospy.loginfo(f"gnss data: {self.gnss}")
                self.pubm = Float64MultiArray()
                self.pubm.data = self.wheel_speeds
                self.rpm_publisher.publish(self.pubm)

                gnss_pub = Float64MultiArray()
                gnss_pub.data = self.gnss
                self.pub_gnss.publish(gnss_pub)

                vcu_data_pub = String()
                vcu_data_pub.data = self.recv_msg
                self.vcu_publisher.publish(vcu_data_pub)   #To send the vcu data to ublox_odom.py
        except Exception as e:
            print(f"hataa!!! {e}")
            pass

    def twist_cb(self,data):
        self.linear_x = data.linear.x
        self.angular_z = data.angular.z

        if self.angular_z <= 0:
            self.angular_dir = 0
        else:
            self.angular_dir = 1

        if self.linear_x <= 0:
            self.linear_dir = 0
        else:
            self.linear_dir =1
    
    def status_cb(self,data:String):
        self.status = data.data

        if self.status == "TELEOP":
            self.led_color = "5"
        elif self.status == "AUTO":
            self.led_color = "1"
        elif self.status == "SUCCESS":
            self.led_color = "6"
        elif self.status == "BLUE":
            self.led_color == "2"
        elif self.status == "OFF":
            self.led_color = "4"

if __name__ == "__main__":
    try:
        ControlVCU()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")

