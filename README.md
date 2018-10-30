# ros2-raspicam
RaspberryPi Camera ROS2 node: porting of UbiquityRobotics/raspicam

## Requirements

 - ROS2 Bouncy
 - [message_filters](https://github.com/ros2/message_filters)
 - [image_common](https://github.com/ros-perception/image_common/tree/ros2)

## Build

Install ROS2 Bouncy on the Raspberry Pi.

Source ROS2 Bouncy.

Build this packages and its dependencies.

    $ mkdir -p ws/src
    $ cd ws/src
    $ git clone https://github.com/ros2/message_filters
    $ git clone -b ros2 https://github.com/ros-perception/image_common
    $ git clone https://github.com/alsora/ros2-raspicam
    $ cd ..
    $ colcon build

## Usage

    $ source install/local_setup.sh
    $ ros2 run raspicam raspicam_node

This will publish `sensor_msgs::msg::CompressedImage` on topic `/image/compressed` at a 30Hz frequency.
