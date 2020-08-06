#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
import message_filters
from robotcar_msgs.msg import Steer
from std_header_msgs.msg import Int8
from std_header_msgs.msg import Float32

class SteerSubscriber(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.steerSub = rospy.Subscriber(self.robot_host + '/steer/get', Steer, self.steerCallback)
        self.steerDegSub = rospy.Subscriber(self.robot_host + '/steer/get/degree', ServoDeg, self.steerDegCallback)
        
        # it's much better to know how big the steering intervall is.
        # normally it`s set to 5 pwm
        self.steerIntervalSub = message_filters.Subscriber(self.robot_host + '/steer/get/intervall', ServoIntervall)
        self.steerIntervalDegSub = message_filters.Subscriber(self.robot_host + '/steer/get/intervall/degree', ServoIntervallDeg)

        tsIntervall = message_filters.ApproximateTimeSynchronizer([self.steerIntervalSub, self.steerIntervalDegSub], 10, 0.1)
        tsIntervall.registerCallback(self.steerIntervallCallback)

        # it's much better to use the Servo-Config instead
        self.steerMinSub = message_filters.Subscriber(self.robot_host + '/steer/get/min', Steer)
        self.steerNeutralSub = message_filters.Subscriber(self.robot_host + '/steer/get/neutral', Steer)
        self.steerMaxSub = message_filters.Subscriber(self.robot_host + '/steer/get/max', Steer)

        tsSteerBorders = message_filters.ApproximateTimeSynchronizer([self.steerMinSub, self.steerNeutralSub, self.steerMaxSub], 10, 0.1)
        tsSteerBorders.registerCallback(self.steerBordersCallback)

        # it's much better to use the Servo-Config instead
        self.steerMinDegSub = message_filters.Subscriber(self.robot_host + '/steer/get/min/degree', ServoDeg)
        self.steerNeutralSub = message_filters.Subscriber(self.robot_host + '/steer/get/neutral/degree', ServoDeg)
        self.steerMaxDegSub = message_filters.Subscriber(self.robot_host + '/steer/get/max/degree', ServoDeg)

        tsSteerBordersDeg = message_filters.ApproximateTimeSynchronizer([self.steerMinDegSub, self.steerNeutralSub, self.steerMaxDegSub], 10, 0.1)
        tsSteerBordersDeg.registerCallback(self.steerBordersDegCallback)

        # maybe it's better to calculate it with the Servo-Config instead
        self.steerRangeSub = message_filters.Subscriber(self.robot_host + '/steer/get/range', ServoRange)
        self.steerRangeDegSub = message_filters.Subscriber(self.robot_host + '/steer/get/range/degree', ServoRangeDeg)

        tsRange = message_filters.ApproximateTimeSynchronizer([self.steerRangeSub, self.steerRangeDegSub], 10, 0.1)
        tsRange.registerCallback(self.steerRangeCallback)

        # Initialize message variables.
        self.enable = False
        self.data = ""

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True

self.steerSub = rospy.Subscriber(self.robot_host + '/steer/get', Steer, self.steerCallback)
        self.steerDegSub = rospy.Subscriber(self.robot_host + '/steer/get/degree', ServoDeg, self.steerDegCallback)
        
        # it's much better to know how big the steering intervall is.
        # normally it`s set to 5 pwm
        self.steerIntervalSub = message_filters.Subscriber(self.robot_host + '/steer/get/intervall', ServoIntervall)
        self.steerIntervalDegSub = message_filters.Subscriber(self.robot_host + '/steer/get/intervall/degree', ServoIntervallDeg)

        tsIntervall = message_filters.ApproximateTimeSynchronizer([self.steerIntervalSub, self.steerIntervalDegSub], 10, 0.1)
        tsIntervall.registerCallback(self.steerIntervallCallback)

        # it's much better to use the Servo-Config instead
        self.steerMinSub = message_filters.Subscriber(self.robot_host + '/steer/get/min', Steer)
        self.steerNeutralSub = message_filters.Subscriber(self.robot_host + '/steer/get/neutral', Steer)
        self.steerMaxSub = message_filters.Subscriber(self.robot_host + '/steer/get/max', Steer)

        tsSteerBorders = message_filters.ApproximateTimeSynchronizer([self.steerMinSub, self.steerNeutralSub, self.steerMaxSub], 10, 0.1)
        tsSteerBorders.registerCallback(self.steerBordersCallback)

        # it's much better to use the Servo-Config instead
        self.steerMinDegSub = message_filters.Subscriber(self.robot_host + '/steer/get/min/degree', ServoDeg)
        self.steerNeutralSub = message_filters.Subscriber(self.robot_host + '/steer/get/neutral/degree', ServoDeg)
        self.steerMaxDegSub = message_filters.Subscriber(self.robot_host + '/steer/get/max/degree', ServoDeg)

        tsSteerBordersDeg = message_filters.ApproximateTimeSynchronizer([self.steerMinDegSub, self.steerNeutralSub, self.steerMaxDegSub], 10, 0.1)
        tsSteerBordersDeg.registerCallback(self.steerBordersDegCallback)

        # maybe it's better to calculate it with the Servo-Config instead
        self.steerRangeSub = message_filters.Subscriber(self.robot_host + '/steer/get/range', ServoRange)
        self.steerRangeDegSub = message_filters.Subscriber(self.robot_host + '/steer/get/range/degree', ServoRangeDeg)

        tsRange = message_filters.ApproximateTimeSynchronizer([self.steerRangeSub, self.steerRangeDegSub], 10, 0.1)
        tsRange.registerCallback(self.steerRangeCallback)
        
    def stop(self):
        """Turn off subscriber."""
        self.enable = False

        self.steerSub.unregister()
        self.steerDegSub.unregister()
        
        self.steerIntervalSub.unregister()
        self.steerIntervalDegSub.unregister()

        self.steerMinSub.unregister()
        self.steerNeutralSub.unregister()
        self.steerMaxSub.unregister()

        self.steerMinDegSub.unregister()
        self.steerNeutralSub.unregister()
        self.steerMaxDegSub.unregister()

        self.steerRangeSub.unregister()
        self.steerRangeDegSub.unregister()

    def steerCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data
        msg = "Got steering angle %s " % self.data.pwm
        rospy.loginfo(rospy.get_caller_id() + msg)

    def steerDegCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got steering angle %s° (Degree)" % data.angle
        rospy.loginfo(rospy.get_caller_id() + msg)

    def steerIntervallCallback(self, intervall, intervallDeg):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got steering intervall %s and %s° (Degree)" % (intervall.intervall_pwm, intervallDeg.intervall_deg)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def steerBordersCallback(self, minData, neutralData, maxData):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got steering angles min %s, neutral %s and max %s" % (minData.pwm, neutralData.pwm, maxData.pwm)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def steerBordersDegCallback(self, minData, neutralData, maxData):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got steering angles min %s° (Degree), neutral %s° (Degree) and max %s° (Degree)" % (minData.angle, neutralData.angle, maxData.angle)
        rospy.loginfo(rospy.get_caller_id() + msg)

    def steerRangeCallback(self, rangeData, rangeDegData):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        msg = "Got steering range %s and %s° (Degree)" % (rangeData.range_pwm, rangeDegData.range_angle)
        rospy.loginfo(rospy.get_caller_id() + msg)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_SteerSubscriber"
    rospy.init_node(node_name, anonymous=False)
    
    steer = SteerSubscriber("robotcar")
    
    # Go to the main loop
    try:
        steer.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        steer.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")