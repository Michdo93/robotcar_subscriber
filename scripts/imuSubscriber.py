#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

class ImuSubscriber(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.imuSub = rospy.Subscriber(self.robot_host + '/imu', Imu, self.imuCallback)
        self.imuRawSub = rospy.Subscriber(self.robot_host + '/imu/raw', Imu, self.imuRawCallback)

        self.accelSub = rospy.Subscriber(self.robot_host + '/imu/accelerometer', Vector3, self.accelerometerCallback)
        self.accelRawSub = rospy.Subscriber(self.robot_host + '/imu/accelerometer/raw', Vector3, self.accelerometerRawCallback)

        self.gyroSub = rospy.Subscriber(self.robot_host + '/imu/gyroscope', Vector3, self.gyroscopeCallback)
        self.gyroRawSub = rospy.Subscriber(self.robot_host + '/imu/gyroscope/raw', Vector3, self.gyroscopeRawCallback)

        self.magnetoSub = rospy.Subscriber(self.robot_host + '/imu/magnetometer', Float64, self.magnetometerCallback)
        self.magnetoRawSub = rospy.Subscriber(self.robot_host + '/imu/magnetometer/raw', Vector3, self.magnetometerRawCallback)

        self.orientationSub = rospy.Subscriber(self.robot_host + '/imu/orientation', Vector3, self.orientationCallback)
        self.orientationDegSub = rospy.Subscriber(self.robot_host + '/imu/orientation/degrees', Vector3, self.orientationDegreesCallback)
        self.orientationRadSub = rospy.Subscriber(self.robot_host + '/imu/orientation/radians', Vector3, self.orientationRadiansCallback)
        self.orientationNorthSub = rospy.Subscriber(self.robot_host + '/imu/orientation/north', Float64, self.orientationNorthCallback)
        
        self.enable = False

        # Initialize message variables.
        self.imuData = ""
        self.imuRawData = ""

        self.accelData = ""
        self.accelRawData = ""

        self.gyroData = ""
        self.gyroRawData = ""

        self.magnetoData = ""
        self.magnetoRawData = ""

        self.orientationData = ""
        self.orientationDegData = ""
        self.orientationRadData = ""
        self.orientationNorthData = ""

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True

        self.imuSub = rospy.Subscriber(self.robot_host + '/imu', Imu, self.imuCallback)
        self.imuRawSub = rospy.Subscriber(self.robot_host + '/imu/raw', Imu, self.imuRawCallback)

        self.accelSub = rospy.Subscriber(self.robot_host + '/imu/accelerometer', Vector3, self.accelerometerCallback)
        self.accelRawSub = rospy.Subscriber(self.robot_host + '/imu/accelerometer/raw', Vector3, self.accelerometerRawCallback)

        self.gyroSub = rospy.Subscriber(self.robot_host + '/imu/gyroscope', Vector3, self.gyroscopeCallback)
        self.gyroRawSub = rospy.Subscriber(self.robot_host + '/imu/gyroscope/raw', Vector3, self.gyroscopeRawCallback)

        self.magnetoSub = rospy.Subscriber(self.robot_host + '/imu/magnetometer', Float64, self.magnetometerCallback)
        self.magnetoRawSub = rospy.Subscriber(self.robot_host + '/imu/magnetometer/raw', Vector3, self.magnetometerRawCallback)

        self.orientationSub = rospy.Subscriber(self.robot_host + '/imu/orientation', Vector3, self.orientationCallback)
        self.orientationDegSub = rospy.Subscriber(self.robot_host + '/imu/orientation/degrees', Vector3, self.orientationDegreesCallback)
        self.orientationRadSub = rospy.Subscriber(self.robot_host + '/imu/orientation/radians', Vector3, self.orientationRadiansCallback)
        self.orientationNorthSub = rospy.Subscriber(self.robot_host + '/imu/orientation/north', Float64, self.orientationNorthCallback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False

        self.imuSub.unregister()
        self.imuRawSub.unregister()

        self.accelSub.unregister()
        self.accelRawSub.unregister()

        self.gyroSub.unregister()
        self.gyroRawSub.unregister()

        self.magnetoSub.unregister()
        self.magnetoRawSub.unregister()

        self.orientationSub.unregister()
        self.orientationDegSub.unregister()
        self.orientationRadSub.unregister()
        self.orientationNorthSub.unregister()

    def imuCallback(self, data):
        self.imuData = data
        msg = "Got type imu: linear-acceleration-pitch: %s, linear-acceleration-roll: %s, linear-acceleration-yaw: %s, gyroscope-pitch: %s, gyroscope-roll: %s, gyroscope-yaw: %s, magnetometer-pitch: %s, magnetometer-roll: %s, magnetometer-yaw: %s" % (self.imuData.linear_acceleration.x, self.imuData.linear_acceleration.y, self.imuData.linear_acceleration.z, self.imuData.angular_velocity.x, self.imuData.angular_velocity.y, self.imuData.angular_velocity.z, self.imuData.orientation.x, self.imuData.orientation.y, self.imuData.orientation.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def imuRawCallback(self, data):
        self.imuRawData = data
        msg = "Got type imu: linear-acceleration-x: %s, linear-acceleration-y: %s, linear-acceleration-z: %s, gyroscope-x: %s, gyroscope-y: %s, gyroscope-z: %s, magnetometer-x: %s, magnetometer-y: %s, magnetometer-z: %s" % (self.imuRawData.linear_acceleration.x, self.imuRawData.linear_acceleration.y, self.imuRawData.linear_acceleration.z, self.imuRawData.angular_velocity.x, self.imuRawData.angular_velocity.y, self.imuRawData.angular_velocity.z, self.imuRawData.orientation.x, self.imuRawData.orientation.y, self.imuRawData.orientation.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def accelerometerCallback(self, data):
        self.accelData = data
        msg = "Got imu/accelerometer: x: %s, y: %s, z: %s" % (self.accelData.x, self.accelData.y, self.accelData.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def accelerometerRawCallback(self, data):
        self.accelRawData = data
        msg = "Got imu/accelerometer/raw: x: %s, y: %s, z: %s" % (self.accelRawData.x, self.accelRawData.y, self.accelRawData.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def gyroscopeCallback(self, data):
        self.gyroData = data
        msg = "Got imu/gyroscope: p: %s, r: %s, y: %s" % (self.gyroData.x, self.gyroData.y, self.gyroData.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def gyroscopeRawCallback(self, data):
        self.gyroRawData = data
        msg = "Got imu/gyroscope/raw: x: %s, y: %s, z: %s" % (self.gyroRawData.x, self.gyroRawData.y, self.gyroRawData.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def magnetometerCallback(self, data):
        self.magnetoData = data
        msg = "Got imu/magnetometer: North: %s" % self.magnetoData.data
        rospy.loginfo(rospy.get_caller_id() + msg)

    def magnetometerRawCallback(self, data):
        self.magnetoRawData = data
        msg = "Got imu/magnetometer/raw: x: %s, y: %s, z: %s" % (self.magnetoRawData.x, self.magnetoRawData.y, self.magnetoRawData.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def orientationCallback(self, data):
        self.orientationData = data
        msg = "Got imu/orientation: p: %s, r: %s, y: %s" % (self.orientationData.x, self.orientationData.y, self.orientationData.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def orientationDegreesCallback(self, data):
        self.orientationDegData = data
        msg = "Got imu/orientation/degrees: p: %s, r: %s, y: %s" % (self.orientationDegData.x, self.orientationDegData.y, self.orientationDegData.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def orientationRadiansCallback(self, data):
        self.orientationRadData = data
        msg = "Got imu/orientation/radians: p: %s, r: %s, y: %s" % (self.orientationRadData.x, self.orientationRadData.y, self.orientationRadData.z)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def orientationNorthCallback(self, data):
        self.orientationNorthData = data
        msg = "Got imu/orientation/north: North: %s" % self.orientationNorthData.data
        rospy.loginfo(rospy.get_caller_id() + msg)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_ImuSubscriber"
    rospy.init_node(node_name, anonymous=False)
    
    imu = ImuSubscriber("robotcar")
    
    # Go to the main loop
    try:
        imu.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        imu.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")