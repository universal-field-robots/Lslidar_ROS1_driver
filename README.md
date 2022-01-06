# lslidar_ch
#version v1.0.0_200825

## version track
Author: leo
### ver2.0.3 leo

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

/***********2020-08-25************/
Original version : LSLIDAR_CH64_V1.0.0_200825_ROSK
Modify  		 :  lslidar_ch64线ROS驱动kinetic版。					

​	luanch文件说明: 

    <param name="channel_num" value="16"/> //通道选择0-63
    <param name="publish_point_cloud" value="true"/> //是否发布所有通道点云
    <param name="publish_channel" value="false"/> //是否发布单通道点云 ，若选择发布则改为true。

查看单通道点云rviz设置：rviz的pointcloud2 的topic 选择/scan_channel。

```
<param name="angle_disable_min" value="0"/>
<param name="angle_disable_max" value="0"/>
单通道点云角度裁剪范围。默认不裁剪。单位角度，30°-150°。
```

Date			 : 2020-08-25