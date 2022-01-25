ROS Installation
-----
[ubuntu](http://wiki.ros.org/Installation/Ubuntu)

Before starting this turorial, please complete installation . This tutorial assumes that Ubuntu is being used.

# lslidar_n301_V3.0

## Description
The `lslidar_n301_V3.0` package is a linux ROS driver for lslidar n301_V3.0/n401_V3.0.

Supported Operating
----
Ubuntu 16.04 Kinetic
Ubuntu 18.04 Melodic

## Connect to the lidar

1. Power the lidar via the included adapter
2. Connect the lidar to an Ethernet port on your computer.
3. Assign the computer IP based on the default DEST IP `192.168.1.125.` <br>`sudo ifconfig eth0 192.168.1.125`（eth0 is the network card name ）<br>

## Compiling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH`  after cloning the package to your workspace. And the normal procedure for compiling a catkin package will work.

```
cd your_work_space<br>
cd src<br>
git clone –b N301_V3.0 https://github.com/lsLIDAR/lslidar_ros/N301_V3.0<br>
catkin_make<br>
source devel/setup.bash/<br>
```

## View Data

1. Launch the provided pointcloud generation launch file.

```
roslaunch lslidar_n301_decoder lslidar_n301.launch
```

1. Launch rviz, with the "laser_link" frame as the fixed frame.

```
rosrun rviz rviz -f laser_link
```

1. In the "displays" panel, click `Add`, click`By topic`,then select `LaserScan`, then press `OK`.

2. In the "Topic" field of the new `LaserScan` tab, enter `/scan`.

## Parameters

The parameters are  in the launch file.

### lslidar_n301_driver

**Parameters**

`frame_id` (`string`, `default: laser_link`)

`device_ip` (`string`, `default: 192.168.1.222`)

By default, the IP address of the device is 192.168.1.222.

`msop_port`(`int`,`default:2368`)

Default value:2368. Data package port.

`difop_port`(`int`,`default:2369`)

Default value:2369.Device package port.

`add_multicast`(`string`,`default:false`)

Switch  to true when lisar is set multicast mode.

`group_ip`(`string`,`default:224.1.1.2`)

### lslidar_n301_decoder

**Parameters**

`child_frame_id` (`string`, `default: laser_link`)

The frame ID entry for the sent messages.

`point_num` (`int`, `default: 2000`)

The points of each frame (default value: 2000) at the scanning rate 5Hz/10Hz/20Hz should be 4000/2000/1000.

`start_angle` (`int`, `default: 0`)

`end_angle` (`int`, `default: 360`)

Points outside this angle will be removed.

`min_range` (`double`, `default: 0.3`)

`max_range` (`double`, `default: 100.0`)

Points outside this range will be removed.

`frequency` (`double`, `default: 10.0`)

The scanning rate(default value:10).Note that the driver does not change the frequency of the sensor. 

`use_gps_ts` (`bool`, `default: false`)

Switch to true when lidar is set GPS time synchronization

`gps_correct` (`bool`, `default: true`)

To fix the singal when the GPS time synchronization is on.

`publish_point_cloud` (`bool`, `default: true`)

The decoder will additionally send out a local point cloud consisting of the points in each revolution.

`filter_scan_point` (`bool`, `default: true`)

To filter point clouds.

**Published Topics**

`lslidar_sweep` (`lslidar_n301_msgs/LslidarN301Sweep`)

The message arranges the points within each sweep based on its scan index and azimuth.

`lslidar_point_cloud` (`sensor_msgs/PointCloud2`)

This is only published when the `publish_point_cloud` is set to `true` in the launch file.

`lslidar_packets` (`lslidar_n301_msgs/LslidarN301Packet`)

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

`scan` (`lslidar_n301_msgs/LaserScan`)

This is published the LaserScan topic.

**Node**

```
roslaunch lslidar_n301_decoder lslidar_n301.launch
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ

## Technical support

Any more question please commit an issue.

Or connect support@lslidar.com



