ROS Installation
-----

[ubuntu](http://wiki.ros.org/Installation/Ubuntu)

Before starting this turorial, please complete installation . This tutorial assumes that Ubuntu is being used.

# lslidar_c16V_2.6

## Description

The `lslidar_c16_V2.6` package is a linux ROS driver for lslidar C16_V2.6.

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
git clone -b C16_v2.6 https://github.com/Lslidar/lslidar_ros.git
catkin_make
source devel/setup.bash
```

## View Data

1. Launch the provided pointcloud generation launch file.

```
roslaunch lslidar_c16_decoder lslidar_c16.launch
```

1. Launch rviz, with the "laser_link" frame as the fixed frame.

```
rosrun rviz rviz -f laser_link
```

1. In the "displays" panel, click `Add`, click`By topic`,then select `pointcloud2`, then press `OK`.

2. In the "Topic" field of the new `pointcloud2` tab, enter `/lslidar_point_cloud`.

### **Parameters**

`device_ip` (`string`, `default: 192.168.1.200`)

By default, the IP address of the device is 192.168.1.200.

`msop_port`(`int`,`default:2368`)

Default value: 2368. Data package port. Modifiable, please keep it consistent with the data package port set by the host computer. 

`difop_port`(`int`,`default:2369`)

Default value:2369.Device package port. Modifiable, please keep it consistent with the device package port set by the host computer. 

`return_mode` (`int`, `default: 1`)

Return mode. Default value: 1. (1 represents single return mode; 2 represents dual return mode)

`time_synchronization` (`bool`, `default: true`)

Default value: true (true: yes; false: no). Whether to open the GPS time synchronization (pre-configuration required). 


### lslidar_c16_driver

`frame_id` (`string`, `default: laser_link`)

Default value: laser_link. Lidar's coordinates name.

`add_multicast`(`bool`,`default: false`)

Default value: false (true: yes; false: no). Whether to switch to the multicast mode. 

`group_ip`(`string`,`default:224.1.1.2`)

Default value: 224.1.1.2. Multicast IP. Enabled when the value of add_multicast is "true".

`rpm` (`int`, `default: 600`)

Lidar's rotate speed. Default value: 600R/M. Modifiable, please keep it consistent with lidar frequency: 5Hz 300R/M，10Hz 600R/M，20Hz 1200R/M

### lslidar_c16_decoder

`min_range` (`double`, `default: 0.15`)

The minimum scanning range. Point cloud data inside this range would be removed. Default value: 0.15 meters.

`max_range` (`double`, `default: 150.0`)

The maximum scanning range. Point cloud data outside this range would be removed. Default value: 150 meters.

`cbMethod` (`bool`, `default: true`)

Transfer from structure center to optical center. Default value: true (ture: yes; false: no)

`config_vert` (`bool`, `default: true`)

Whether to open lidar vertical angle configuration calibration. Default value: true (ture: yes; false: no)

`print_vert` (`bool`, `default: false`)

Whether to open lidar vertical angle print. Default value: false (ture: yes; false: no)

`distance_unit` (`double` `default: 0.25`)

Lidar's distance resolution. Default value: 0.25 meters. Do not modify.

**Published Topics**

`lslidar_point_cloud` (`sensor_msgs/PointCloud2`)

This is published the lslidar_point_cloud topic.

`lslidar_packets` (`lslidar_c16_msgs/Lslidarc16Packet`)

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

`scan` (`lslidar_c16_msgs/LaserScan`)

This is only published when the `publish_scan`is set to `true` in the launch file.

**Node**

```
roslaunch lslidar_c16_decoder lslidar_c16.launch
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ

## Technical support

Any more question please commit an issue.

Or connect support@lslidar.com







****
