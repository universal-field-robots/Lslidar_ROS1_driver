# lslidar_ch
#version v2.0.0_211025

## version track
Author: leo



## Description
The `lslidar_ch` package is a linux ROS driver for lslidar ch.
The package is tested on Ubuntu 16.04 with ROS kinetic.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
```

## Example Usage

### lslidar_ch_decoder

**Parameters**

`lidar_ip` (`string`, `default: 192.168.1.200`)

By default, the IP address of the device is 192.168.1.200.

`frame_id` (`string`, `default: laser_link`)

The frame ID entry for the sent messages.

**Published Topics**

`lslidar_point_cloud`

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

### lslidar_ch_decoder

**Parameters**

`min_range` (`double`, `0.3`)

`max_range` (`double`, `200.0`)

Points outside this range will be removed.

`frequency` (`frequency`, `10.0`)

Note that the driver does not change the frequency of the sensor. 

`publish_point_cloud` (`bool`, `true`)

If set to true, the decoder will additionally send out a local point cloud consisting of the points in each revolution.

**Published Topics**

`lslidar_sweep` (`lslidar_ch_msgs/LslidarChSweep`)

The message arranges the points within each sweep based on its scan index and azimuth.

`lslidar_point_cloud` (`sensor_msgs/PointCloud2`)

This is only published when the `publish_point_cloud` is set to `true` in the launch file.

**Node**

```
roslaunch lslidar_ch_decoder lslidar_ch.launch --screen
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ


## Bug Report

##Version changes

/***********2021-06-10************/
Original version : LSLIDAR_CH64W_V1.0.1_210610_ROS
Modify  		 : 根据ch64w线雷达协议更新 lslidar_ch64w线ROS驱动。				

Date			 : 2021-06-10



/***********2021-10-25************/
Original version : LSLIDAR_CH64W_V1.0.1_210610_ROS

Revised version  :LSLIDAR_CH64W_V2.0.0_211025_ROS

Modify  		 :  根据ch64w线雷达新协议更新ch64w线ROS驱动。	

1，新增离线播放pcap文件功能。

2，增加laserscan类型话题发布。

3，点云数据格式统一，增加ring和time。			

Date			 : 2021-10-25