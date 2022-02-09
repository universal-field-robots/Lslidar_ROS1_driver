ROS Installation
-----

[ubuntu](http://wiki.ros.org/Installation/Ubuntu)

Before starting this turorial, please complete installation . This tutorial assumes that Ubuntu is being used.

# lslidar_m10_net_v1.0
[Customer service entrance](https://1893520.s5.udesk.cn/im_client/?web_plugin_id=502)
## Description

The `lslidar_m10` package is a linux ROS driver for lslidar m10_net_v1.0

Supported Operating
----

Ubuntu 16.04 Kinetic
Ubuntu 18.04 Melodic

## Connect to the lidar

1. Power the lidar via the included adapter
2. Connect the lidar to an Ethernet port on your computer.
3. Assign the computer IP based on the default DEST IP `192.168.1.102.` <br>`sudo ifconfig eth0 192.168.1.102`（eth0 is the network card name ）<br>

## Compiling

This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH`  after cloning the package to your workspace. And the normal procedure for compiling a catkin package will work.

```
cd your_work_space
cd src
git clone -b M10_net_v1.0 https://github.com/Lslidar/lslidar_ros.git
catkin_make
source devel/setup.bash
```

## View Data

1. Launch the provided pointcloud generation launch file.

```
roslaunch lslidar_m10_decoder lslidar_m10.launch
```

1. Launch rviz, with the "laser_link" frame as the fixed frame.

```
rosrun rviz rviz -f laser_link
```

1. In the "displays" panel, click `Add`, click`By topic`,then select `LaserScan`, then press `OK`.

2. In the "Topic" field of the new `LaserScan` tab, enter `/scan`.

## **Parameters**

### lslidar_m10_driver

`frame_id` (`string`, `default: laser_link`)

Default value: laser_link. Lidar coordinates name.

`device_ip` (`string`, `default: 192.168.1.200`)

By default, the IP address of the device is 192.168.1.200.

`device_port`(`int`, `default:2368`)

Default value:2368. Data package port. 


### lslidar_m10_decoder

`child_frame_id` (`string`, `default: laser_link`)

Lidar coordinates name. Default value: laser_link

`scan_topic` (`string`, `default: scan`)

Lidar data topic name. Default value: scan.

`point_num` (`int`, `default: 948`)

Point cloud number at ratate speed of 10Hz. Default value: 948.

`angle_disable_min` (`double`, `default: 0.0`)

Disable agle starting value. Default value: 0.

`angle_disable_max` (`double`, `default: 0.0`)

Disable agle end value. Default value: 0. Point cloud data between the starting and end angle would be removed.

`min_range` (`double`, `default: 0.3`)

The minimum scanning range. Point cloud data inside this range would be removed. Default value: 0.3 meters.

`max_range` (`double`, `default: 100.0`)

The maximum scanning range. Point cloud data outside this range would be removed. Default value: 100 meters.

**Published Topics**

`lslidar_packets` (`lslidar_m10_msgs/Lslidarm10Packet`)

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

`scan` (`lslidar_m10_msgs/LaserScan`)

This is published the LaserScan topic.

**Node**

```
roslaunch lslidar_m10_decoder lslidar_m10.launch
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ

## Technical support

Any more question please commit an issue.

you can contact support@lslidar.com
or Enter our live chat window
[Customer service entrance](https://1893520.s5.udesk.cn/im_client/?web_plugin_id=502)



****
