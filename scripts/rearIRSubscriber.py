#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from sensor_msgs.msg import Range
from robotcar_msgs.msg import RelativeVelocity

class RearIRSubscriber(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.sub = rospy.Subscriber(self.robot_host + '/infrared/rear/distance', Range, self.callback)
        self.velocitySub = rospy.Subscriber(self.robot_host + '/infrared/rear/relative_velocity', RelativeVelocity, self.velocityCallback)

        # Initialize message variables.
        self.enable = False
        self.data = ""

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber(self.robot_host + '/infrared/rear/distance', Range, self.callback)
        self.velocitySub = rospy.Subscriber(self.robot_host + '/infrared/rear/relative_velocity', RelativeVelocity, self.velocityCallback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()
        self.velocitySub.unregister()

    def callback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data
        msg = "Got type %s with FoV %s and Min-Range %s and Max-Range %s and measured Range %s" % (self.data.radiation_type, self.data.field_of_view, self.data.min_range, self.data.max_range, self.data.range)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def velocityCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got type %s with FoV %s and measured Relative Velocity %s" % (data.radiation_type, data.field_of_view, data.relative_velocity)
        rospy.loginfo(rospy.get_caller_id() + msg)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_RearIRSubscriber"
    rospy.init_node(node_name, anonymous=False)
    
    infrared = RearIRSubscriber("robotcar")
    
    # Go to the main loop
    try:
        infrared.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        infrared.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")