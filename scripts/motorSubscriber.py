#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from robotcar_msgs.msg import Motor
from robotcar_msgs.msg import DutyCycle
from robotcar_msgs.msg import Rpm
from robotcar_msgs.msg import Velocity

class MotorSubscriber(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host

        self.motorSub = rospy.Subscriber(self.robot_host + '/motor/get', Motor, self.motorCallback)
        
        self.motorMSSub = rospy.Subscriber(self.robot_host + '/motor/get/ms', Velocity, self.msCallback)
        self.motorKMHSub = rospy.Subscriber(self.robot_host + '/motor/get/kmh', Velocity, self.kmhCallback)
        self.motorMPHSub = rospy.Subscriber(self.robot_host + '/motor/get/mph', Velocity, self.mphCallback)
        
        self.motorPWMSub = rospy.Subscriber(self.robot_host + '/motor/get/pwm', DutyCycle, self.pwmCallback)
        self.motorRPMSub = rospy.Subscriber(self.robot_host + '/motor/get/rpm', Rpm, self.rpmCallback)

        # Initialize message variables.
        self.enable = False
        self.data = ""

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.motorSub = rospy.Subscriber(self.robot_host + '/motor/get', Motor, self.motorCallback)
        
        self.motorMSSub = rospy.Subscriber(self.robot_host + '/motor/get/ms', Velocity, self.msCallback)
        self.motorKMHSub = rospy.Subscriber(self.robot_host + '/motor/get/kmh', Velocity, self.kmhCallback)
        self.motorMPHSub = rospy.Subscriber(self.robot_host + '/motor/get/mph', Velocity, self.mphCallback)
        
        self.motorPWMSub = rospy.Subscriber(self.robot_host + '/motor/get/pwm', DutyCycle, self.pwmCallback)
        self.motorRPMSub = rospy.Subscriber(self.robot_host + '/motor/get/rpm', Rpm, self.rpmCallback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        
        self.motorSub.unregister()

        self.motorMSSub.unregister()
        self.motorKMHSub.unregister()
        self.motorMPHSub.unregister()
        
        self.motorPWMSub.unregister()
        self.motorRPMSub.unregister()

    def motorCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data
        msg = "Got motor rate %s and motor direction %s" % (self.data.rate, self.data.direction)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def msCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got %s m/s" % data.velocity
        rospy.loginfo(rospy.get_caller_id() + msg)

    def kmhCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got %s km/h" % data.velocity
        rospy.loginfo(rospy.get_caller_id() + msg)

    def mphCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got %s mp/h" % data.velocity
        rospy.loginfo(rospy.get_caller_id() + msg)

    def pwmCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got pwm value %s" % data.pwm
        rospy.loginfo(rospy.get_caller_id() + msg)

    def rpmCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got %s rpm" % data.rpm
        rospy.loginfo(rospy.get_caller_id() + msg)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_MotorSubscriber"
    rospy.init_node(node_name, anonymous=False)
    
    motor = MotorSubscriber("robotcar")
    
    # Go to the main loop
    try:
        motor.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        motor.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")