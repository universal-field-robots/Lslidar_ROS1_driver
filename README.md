# lslidar_ch16
#version v2.0.3_210824

## version track
Author: leo

### ver1.0.0 leo

## Description
The `lslidar_ch16` package is a linux ROS driver for lslidar ch.
The package is tested on Ubuntu 16.04 with ROS kinetic.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
```

## Example Usage

### lslidar_ch16_decoder

**Parameters**

`lidar_ip` (`string`, `default: 192.168.1.200`)

By default, the IP address of the device is 192.168.1.200.

`frame_id` (`string`, `default: laser_link`)

The frame ID entry for the sent messages.

**Published Topics**

`lslidar_point_cloud`

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

### lslidar_ch16_decoder

**Parameters**

`min_range` (`double`, `0.3`)

`max_range` (`double`, `200.0`)

Points outside this range will be removed.

`frequency` (`frequency`, `10.0`)

Note that the driver does not change the frequency of the sensor. 

`publish_point_cloud` (`bool`, `true`)

If set to true, the decoder will additionally send out a local point cloud consisting of the points in each revolution.

**Published Topics**

`lslidar_sweep` (`lslidar_ch16_msgs/LslidarChSweep`)

The message arranges the points within each sweep based on its scan index and azimuth.

`lslidar_point_cloud` (`sensor_msgs/PointCloud2`)

This is only published when the `publish_point_cloud` is set to `true` in the launch file.

**Node**

```
roslaunch lslidar_ch16_decoder lslidar_ch16.launch --screen
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ


## Bug Report

/***********2020-04-03****************/
Original version : lslidar_ch16_v1.01_191012
Revised version  : lslidar_ch16_v2.01_200403
Modify           : 修改为高精度版本，修改距离解析(距离用3个字节存放)
Author           : zx
Date             : 2020-04-03



/***********2021-08-24************/
Original version : lslidar_ch16_v2.0.2_200722
Original version : LSLIDAR_CH16_V2.0.3_210824_ROS
Modify  		 :  		

1、点云中的点增加ring和time信息。

2、新增laserscan类型话题发布。若需发布，修改lslidar_ch16.launch文件

```xml
<param name="publish_laserscan" value="false"/> 
!--改为--
<param name="publish_laserscan" value="true"/> 
```

3、棱镜角度从设备包获取解析。	

4、优化在同一工作空间下编译不同雷达驱动报错问题。		

Date			 : 2021-08-24




