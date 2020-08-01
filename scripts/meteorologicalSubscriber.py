#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import Temperature

class MeteorologicalSubscriber(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.pressureSub = rospy.Subscriber(self.robot_host + '/meteorological/pressure', FluidPressure, self.pressureCallback)
        self.humditiySub = rospy.Subscriber(self.robot_host + '/meteorological/humidity', RelativeHumidity, self.humidityCallback)
        self.temperatureSub = rospy.Subscriber(self.robot_host + '/meteorological/temperature', Temperature, self.temperatureCallback)

        # Initialize message variables.
        self.enable = False
        self.pressureData = ""
        self.humidityData = ""
        self.temperatureData = ""

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.pressureSub = rospy.Subscriber(self.robot_host + '/meteorological/pressure', FluidPressure, self.pressureCallback)
        self.humditiySub = rospy.Subscriber(self.robot_host + '/meteorological/humidity', RelativeHumidity, self.humidityCallback)
        self.temperatureSub = rospy.Subscriber(self.robot_host + '/meteorological/temperature', Temperature, self.temperatureCallback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.pressureSub.unregister()
        self.humditiySub.unregister()
        self.temperatureSub.unregister()

    def pressureCallback(self, data):
        self.pressureData = data
        msg = "Got Pressure: %s Pascal with variance %s" % (self.pressureData.fluid_pressure, self.pressureData.variance)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def humidityCallback(self, data):
        self.humidityData = data
        msg = "Got Relative Humidity: %s rH with variance %s" % (self.humidityData.relative_humidity, self.humidityData.variance)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def temperatureCallback(self, data):
        self.temperatureData = data
        msg = "Got Temperature: %s °C with variance %s" % (self.temperatureData.temperature, self.temperatureData.variance)
        rospy.loginfo(rospy.get_caller_id() + msg)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_MeteorologicalSubscriber"
    rospy.init_node(node_name, anonymous=False)
    
    meteorological = MeteorologicalSubscriber("robotcar")
    
    # Go to the main loop
    try:
        meteorological.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        meteorological.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")