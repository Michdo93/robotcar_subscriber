# robotcar_subscriber


Gives an example how you can use Subscribers for receiving sensor informations from the [RobotCar](https://github.com/Michdo93/robotcar). At first you have to make sure that the roscore is running and the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) is publishing the sensor informations.

The String variable `robot_host` uses the hostname of one RobotCar. As example it could be `robotcar`.


## CompressedImageSubscriber Node

It subscribes the compressed image informations from the [raspicam_node](https://github.com/Michdo93/raspicam_node) respectively the raspicam.

|                 Topic Address                |            Message Type       |
|--------------------------------------------- | ------------------------------|
|robot_host + /raspicam/image/compressed       | [sensor_msgs/CompressedImage](http://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html)  |

You can run it with `rosrun robotcar_subscriber compressedImageSubscriber.py`

## FrontIRSubscriber Node

It subscribes informations from the FrontInfrared Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front infrared sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /infrared/front/distance                  | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /infrared/front/relative_velocity         | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber frontIRSubscriber.py`

## FrontLeftUltrasonicSubscriber Node

It subscribes informations from the FrontLeftUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the left front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/left/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/front/left/relative_velocity  | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber frontLeftUltrasonicSubscriber.py`

## FrontRightUltrasonicSubscriber Node

It subscribes informations from the FrontRightUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the right front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/right/distance          | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/front/right/relative_velocity | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber frontRightUltrasonicSubscriber.py`

## FrontTofSubscriber Node

It subscribes informations from the FrontTimeOfFlight Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front time-of-flight sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /time_of_flight/front/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /time_of_flight/front/relative_velocity   | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber frontToFSubscriber.py`

## FrontUltrasonicSubscriber Node

It subscribes informations from the FrontUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/distance                | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/front/relative_velocity       | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber frontUltrasonicSubscriber.py`

## ImuSubscriber Node

It subscribes informations from the Imu Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively from the Imu of the Sense HAT.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /imu                                      | [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)                |
|robot_host + /imu/raw                                  | [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)                |
|robot_host + /imu/accelerometer                        | [robotcar_msgs/Accelerometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Accelerometer.md)    |
|robot_host + /imu/accelerometer/raw                    | [robotcar_msgs/Accelerometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Accelerometer.md)    |
|robot_host + /imu/gyroscope                            | [robotcar_msgs/Gyroscope](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Gyroscope.md)        |
|robot_host + /imu/gyroscope/raw                        | [robotcar_msgs/Gyroscope](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Gyroscope.md)        |
|robot_host + /imu/magenetometer                        | [robotcar_msgs/Magnetometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Magnetometer.md)     |
|robot_host + /imu/magenetometer/raw                    | [robotcar_msgs/Orientation](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Orientation.md)      |
|robot_host + /imu/orientation                          | [robotcar_msgs/Orientation](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Orientation.md)      |
|robot_host + /imu/orientation/degrees                  | [robotcar_msgs/Orientation](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Orientation.md)      |
|robot_host + /imu/orientation/radians                  | [robotcar_msgs/Orientation](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Orientation.md)      |
|robot_host + /imu/orientation/north                    | [robotcar_msgs/Magnetometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Magnetometer.md)     |
        
You can run it with `rosrun robotcar_subscriber imuSubscriber.py`

## MeteorologicalSubscriber Node

It subscribes informations from the Meteorological Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the meteorological sensors of the Sense HAT.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /meteorological/pressure                  | [sensor_msgs/FluidPressure](http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html)     |
|robot_host + /meteorological/humidity                  | [sensor_msgs/RelativeHumidity](http://docs.ros.org/en/api/sensor_msgs/html/msg/RelativeHumidity.html)  |
|robot_host + /meteorological/temperature               | [sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)       |

You can run it with `rosrun robotcar_subscriber meteorologicalSubscriber.py`

## MotorSubscriber Node

It subscribes informations from the Motor Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the motor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /motor/get                                | [robotcar_msgs/Motor](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Motor.md)            |
|robot_host + /motor/get/ms                             | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
|robot_host + /motor/get/kmh                            | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
|robot_host + /motor/get/mph                            | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
|robot_host + /motor/get/pwm                            | [robotcar_msgs/DutyCycle](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/DutyCycle.md)        |
|robot_host + /motor/get/rpm                            | [robotcar_msgs/Rpm](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Rpm.md)              |
        
You can run it with `rosrun robotcar_subscriber motorSubscriber.py`

## RearIRSubscriber Node

It subscribes informations from the RearInfrared Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear infrared sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /infrared/rear/distance                   | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /infrared/rear/relative_velocity          | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber rearIRSubscriber.py`

## RearLeftUltrasonicSubscriber Node

It subscribes informations from the RearLeftUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the left rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/left/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/rear/left/relative_velocity   | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber rearLeftUltrasonicSubscriber.py`

## RearRightUltrasonicSubscriber Node

It subscribes informations from the RearRightUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the right rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/right/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/rear/right/relative_velocity  | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber rearRightUltrasonicSubscriber.py`

## RearToFSubscriber Node

It subscribes informations from the RearTimeOfFlight Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear time-of-flight sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /time_of_flight/rear/distance             | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /time_of_flight/rear/relative_velocity    | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber rearToFSubscriber.py`

## RearUltrasonicSubscriber Node

It subscribes informations from the RearUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/distance                 | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
|robot_host + /ultrasonic/rear/relative_velocity        | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it with `rosrun robotcar_subscriber rearUltrasonicSubscriber.py`

## SteerSubscriber Node

It subscribes informations from the Steer Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the steer servo motor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /steer/get                                | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
|robot_host + /steer/get/intervall                      | [robotcar_msgs/ServoIntervall](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoIntervall.md)   |
|robot_host + /steer/get/intervall/degree               | [robotcar_msgs/ServoIntervallDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoIntervallDeg.md)|
|robot_host + /steer/get/min                            | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
|robot_host + /steer/get/neutral                        | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
|robot_host + /steer/get/max                            | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
|robot_host + /steer/get/min/degree                     | [robotcar_msgs/ServoDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoDeg.md)         |
|robot_host + /steer/get/neutral/degree                 | [robotcar_msgs/ServoDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoDeg.md)         |
|robot_host + /steer/get/max/degree                     | [robotcar_msgs/ServoDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoDeg.md)         |
|robot_host + /steer/get/range                          | [robotcar_msgs/ServoRange](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoRange.md)       |
|robot_host + /steer/get/range/degree                   | [robotcar_msgs/ServoRangeDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoRangeDeg.md)    |
        
You can run it with `rosrun robotcar_subscriber steerSubscriber.py`
