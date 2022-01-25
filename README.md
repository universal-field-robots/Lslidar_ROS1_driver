ROS Installation
-----

[ubuntu](http://wiki.ros.org/Installation/Ubuntu)

Before starting this turorial, please complete installation . This tutorial assumes that Ubuntu is being used.

# lslidar_N10_v1.0

## Description

The `lslidar_N10_v1.0` package is a linux ROS driver for lslidar N10_v1.0.

Supported Operating
----

Ubuntu 16.04 Kinetic
Ubuntu 18.04 Melodic

## Connect to the lidar

1. Connect the lidar to the included USB adapter board
2. Use a USB cable to connect the USB adapter board to the computer USB port
3. Use `sudo ls -l /dev/ttyUSB*` command to find serial port name <br>Use `sudo chmod 777 /dev/serialportname` to give permissions to the serial port<br>

## Compiling

This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH`  after cloning the package to your workspace. And the normal procedure for compiling a catkin package will work.

```
cd your_work_space
cd src
git clone -b N10_v1.0 https://github.com/Lslidar/lslidar_ros.git
catkin_make
source devel/setup.bash
```

## View Data

1. Launch the provided pointcloud generation launch file.

```
roslaunch lsn10 lsn10.launch
```

1. Launch rviz, with the "laser_link" frame as the fixed frame.

```
rosrun rviz rviz -f laser_link
```

1. In the "displays" panel, click `Add`, click`By topic`,then select `LaserScan`, then press `OK`.

2. In the "Topic" field of the new `LaserScan` tab, enter `/scan`.

**Parameters**

`scan_topic` (`string`, `default: scan`)

Lidar data topic name. Default value: scan.

`frame_id` (`string`, `default: laser_link`)

Default value: laser_link. Lidar coordinates name.

`serial_port` (`string`, `default: /dev/ttyUSB0`)

Serial port. Default value: /dev/ttyUSB0

`baud_rate` (`int`, `default: 230400`)

Serial port baud rate. Default value: 230400

`angle_min` (`double`, `default: 0.0`)

Scan angle starting value. Default value: 0.

`angle_max` (`double`, `default: 360.0`)

Scan angle end value. Default value: 360. Point cloud data outside the starting and end angle would be removed.

`range_min` (`double`, `default: 0.02`)

The minimum scanning range. Point cloud data inside this range would be removed. Default value: 0.0 meter.

`range_max` (`double`, `default: 150.0`)

The maximum scanning range. Point cloud data outside this range would be removed. Default value: 100 meters.

**Published Topics**

`parameter descriptions`

this is publish the truncated lidar data value

`scan` (`lslidar_n10msgs/LaserScan`)

This is published the LaserScan topic.

**Node**

```
roslaunch lsn10 lsn10.launch
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ

## Technical support

Any more question please commit an issue.

Or connect support@lslidar.com





****
