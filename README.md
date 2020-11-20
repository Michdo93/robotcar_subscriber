# robotcar_subscriber


Gives an example how you can use Subscribers for receiving sensor informations from the [RobotCar](https://github.com/Michdo93/robotcar). At first you have to make sure that the roscore is running and the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) is publishing the sensor informations.

The String variable `robot_host` uses the hostname of one RobotCar. As example it could be `robotcar`.


## CompressedImageSubscriber Node

It subscribes the compressed image informations from the [raspicam_node](https://github.com/Michdo93/raspicam_node) respectively the raspicam.

|                 Topic Address                |            Message Type       |
|--------------------------------------------- | ------------------------------|
|robot_host + /raspicam/image/compressed       | [sensor_msgs/CompressedImage](http://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html)  |

## FrontIRSubscriber Node

It subscribes informations from the FrontInfrared Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front infrared sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /infrared/front/distance                  | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /infrared/front/relative_velocity         | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## FrontLeftUltrasonicSubscriber Node

It subscribes informations from the FrontLeftUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the left front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/left/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/front/left/relative_velocity  | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## FrontRightUltrasonicSubscriber Node

It subscribes informations from the FrontRightUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the right front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/right/distance          | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/front/right/relative_velocity | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## FrontTofSubscriber Node

It subscribes informations from the FrontTimeOfFlight Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front time-of-flight sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /time_of_flight/front/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /time_of_flight/front/relative_velocity   | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## FrontUltrasonicSubscriber Node

It subscribes informations from the FrontUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/distance                | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/front/relative_velocity       | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## ImuSubscriber Node

It subscribes informations from the Imu Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively from the Imu of the Sense HAT.

        self.imuSub = rospy.Subscriber(self.robot_host + '/imu', Imu, self.imuCallback)
        self.imuRawSub = rospy.Subscriber(self.robot_host + '/imu/raw', Imu, self.imuRawCallback)

        self.accelSub = rospy.Subscriber(self.robot_host + '/imu/accelerometer', Accelerometer, self.accelerometerCallback)
        self.accelRawSub = rospy.Subscriber(self.robot_host + '/imu/accelerometer/raw', Accelerometer, self.accelerometerRawCallback)

        self.gyroSub = rospy.Subscriber(self.robot_host + '/imu/gyroscope', Gyroscope, self.gyroscopeCallback)
        self.gyroRawSub = rospy.Subscriber(self.robot_host + '/imu/gyroscope/raw', Gyroscope, self.gyroscopeRawCallback)

        self.magnetoSub = rospy.Subscriber(self.robot_host + '/imu/magnetometer', Magnetometer, self.magnetometerCallback)
        self.magnetoRawSub = rospy.Subscriber(self.robot_host + '/imu/magnetometer/raw', Orientation, self.magnetometerRawCallback)

        self.orientationSub = rospy.Subscriber(self.robot_host + '/imu/orientation', Orientation, self.orientationCallback)
        self.orientationDegSub = rospy.Subscriber(self.robot_host + '/imu/orientation/degrees', Orientation, self.orientationDegreesCallback)
        self.orientationRadSub = rospy.Subscriber(self.robot_host + '/imu/orientation/radians', Orientation, self.orientationRadiansCallback)
        self.orientationNorthSub = rospy.Subscriber(self.robot_host + '/imu/orientation/north', Magnetometer, self.orientationNorthCallback)

## MeteorologicalSubscriber Node

It subscribes informations from the Meteorological Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the meteorological sensors of the Sense HAT.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /meteorological/pressure                  | [sensor_msgs/FluidPressure](http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html)     |
|robot_host + /meteorological/humidity                  | [sensor_msgs/RelativeHumidity](http://docs.ros.org/en/api/sensor_msgs/html/msg/RelativeHumidity.html)  |
|robot_host + /meteorological/temperature               | [sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)       |

## MotorSubscriber Node

It subscribes informations from the Motor Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the motor.

        self.motorSub = rospy.Subscriber(self.robot_host + '/motor/get', Motor, self.motorCallback)
        
        self.motorMSSub = rospy.Subscriber(self.robot_host + '/motor/get/ms', Velocity, self.msCallback)
        self.motorKMHSub = rospy.Subscriber(self.robot_host + '/motor/get/kmh', Velocity, self.kmhCallback)
        self.motorMPHSub = rospy.Subscriber(self.robot_host + '/motor/get/mph', Velocity, self.mphCallback)
        
        self.motorPWMSub = rospy.Subscriber(self.robot_host + '/motor/get/pwm', DutyCycle, self.pwmCallback)
        self.motorRPMSub = rospy.Subscriber(self.robot_host + '/motor/get/rpm', Rpm, self.rpmCallback)

## RearIRSubscriber Node

It subscribes informations from the RearInfrared Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear infrared sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /infrared/rear/distance                   | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /infrared/rear/relative_velocity          | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## RearLeftUltrasonicSubscriber Node

It subscribes informations from the RearLeftUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the left rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/left/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/rear/left/relative_velocity   | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## RearRightUltrasonicSubscriber Node

It subscribes informations from the RearRightUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the right rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/right/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/rear/right/relative_velocity  | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## RearToFSubscriber Node

It subscribes informations from the RearTimeOfFlight Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear time-of-flight sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /time_of_flight/rear/distance             | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /time_of_flight/rear/relative_velocity    | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## RearUltrasonicSubscriber Node

It subscribes informations from the RearUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/distance                 | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/rear/relative_velocity        | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

## SteerSubscriber Node

It subscribes informations from the Steer Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the steer servo motor.

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
