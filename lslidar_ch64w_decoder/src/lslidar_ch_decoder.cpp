/*
 * This file is part of lslidar_ch driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <lslidar_ch_decoder/lslidar_ch_decoder.h>
#include <std_msgs/Int8.h>

using namespace std;

namespace lslidar_ch_decoder {
    LslidarChDecoder::LslidarChDecoder(
            ros::NodeHandle &n, ros::NodeHandle &pn) :
            nh(n),
            pnh(pn),
            publish_point_cloud(true),
            is_first_sweep(true),
            last_azimuth(0.0),
            sweep_start_time(0.0),
            sweep_data(new lslidar_ch64w_msgs::LslidarChScan()) {
        packet_timeStamp.sec = 0;
        packet_timeStamp.nsec = 0;
        packet_end_timeStamp.sec = 0;
        packet_end_timeStamp.nsec = 0;
        return;
    }

    bool LslidarChDecoder::loadParameters() {
        pnh.param<double>("min_range", min_range, 0.5);
        pnh.param<double>("max_range", max_range, 100.0);
        pnh.param<double>("angle_disable_min", angle_disable_min, 0.0);
        pnh.param<double>("angle_disable_max", angle_disable_max, 0.0);
        pnh.param<double>("pulse_repetition_rate", pulse_repetition_rate, 60.0);
        pnh.param<int>("frequency", frequency, 10);
        pnh.param<string>("pointcloud_topic", pointcloud_topic, "lslidar_point_cloud");
        pnh.param<string>("scan_channel_topic", scan_channel_topic, "scan_channel");
        pnh.param<bool>("publish_point_cloud", publish_point_cloud, true);
        pnh.param<bool>("publish_laserscan", publish_laserscan, false);
        pnh.param<int>("channel_num", channel_num, 1);
        //pnh.param<string>("fixed_frame_id", fixed_frame_id, "map");
        pnh.param<string>("frame_id", frame_id, "lslidar");
        pnh.param<bool>("time_synchronization", time_synchronization_, false);
        ROS_INFO("Using GPS timestamp or not %d", time_synchronization_);
        angle_disable_min = angle_disable_min * DEG_TO_RAD;
        angle_disable_max = angle_disable_max * DEG_TO_RAD;
        if (publish_laserscan) {
            if (channel_num < 0) {
                channel_num = 0;
                ROS_WARN("channel_num outside of the index, select channel 0 instead!");
            } else if (channel_num > 127) {
                channel_num = 127;
                ROS_WARN("channel_num outside of the index, select channel 63 instead!");
            }
            ROS_INFO("select channel num: %d", channel_num);
        }
        switch (frequency) {
            case 5:
                horizontal_angle_resolution = DEG2RAD(0.06);
                break;
            case 20:
                horizontal_angle_resolution = DEG2RAD(0.24);
                break;
            default:
                horizontal_angle_resolution = DEG2RAD(0.12);
        }

        return true;
    }

    bool LslidarChDecoder::createRosIO() {
        packet_sub = nh.subscribe<lslidar_ch64w_msgs::LslidarChPacket>(
                "lslidar_packet_64w", 100, &LslidarChDecoder::packetCallback, this);
        point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
                pointcloud_topic, 10);
        laserscan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);

        return true;
    }


    bool LslidarChDecoder::initialize() {
        point_time_diff = 1 / (pulse_repetition_rate * 1000 * 32);  // s
        for (int i = 0; i < 4; ++i) {
            prism_angle[i] = i * 0.35;
        }
        for (int j = 0; j < 128; ++j) {
            //右边
            if (j / 4 % 2 == 0) {
                theat1_s[j] = sin((-25 + floor(j / 8) * 2.5) * M_PI / 180);
                theat2_s[j] = sin(prism_angle[j % 4] * M_PI / 180);
                theat1_c[j] = cos((-25 + floor(j / 8) * 2.5) * M_PI / 180);
                theat2_c[j] = cos(prism_angle[j % 4] * M_PI / 180);
            }
            //左边
            else {
                theat1_s[j] = sin((-24 + floor(j / 8) * 2.5) * M_PI / 180);
                theat2_s[j] = sin(prism_angle[j % 4] * M_PI / 180);
                theat1_c[j] = cos((-24 + floor(j / 8) * 2.5) * M_PI / 180);
                theat2_c[j] = cos(prism_angle[j % 4] * M_PI / 180);
            }
        }
        if (!loadParameters()) {
            ROS_ERROR("Cannot load all required parameters...");
            return false;
        }

        if (!createRosIO()) {
            ROS_ERROR("Cannot create ROS I/O...");
            return false;
        }
        return true;
    }

    void LslidarChDecoder::publishPointCloud() {

        pcl::PointCloud<VPoint>::Ptr point_cloud(new pcl::PointCloud<VPoint>);
        point_cloud->header.frame_id = frame_id;
        point_cloud->height = 1;

        // The first and last point in each scan is ignored, which
        // seems to be corrupted based on the received data.
        // TODO: The two end points should be removed directly
        //    in the scans.
        double timestamp = ros::Time::now().toSec();

        size_t j;

        //pcl::PointXYZI point;
        VPoint point;
        if (sweep_data->points.size() > 0) {

            if (time_synchronization_) {
                point_cloud->header.stamp = pcl_conversions::toPCL(packet_timeStamp);
            } else {
                point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
            }
            for (j = 1; j < sweep_data->points.size() - 1; ++j) {
//                if ((sweep_data->points[j].azimuth > angle_disable_min) and
//                    (sweep_data->points[j].azimuth < angle_disable_max)) {
//                    continue;
//                }
                point.time = sweep_data->points[j].time;
                point.x = sweep_data->points[j].x;
                point.y = sweep_data->points[j].y;
                point.z = sweep_data->points[j].z;
                point.intensity = sweep_data->points[j].intensity;
                point.ring = sweep_data->points[j].line;
                point_cloud->points.push_back(point);
                ++point_cloud->width;
            }
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            point_cloud_pub.publish(pc_msg);
        }
        return;
    }

    void LslidarChDecoder::publishLaserScan() {
        sensor_msgs::LaserScan::Ptr scan_msg(new sensor_msgs::LaserScan);
        scan_msg->header.frame_id = frame_id;
        scan_msg->angle_min = DEG2RAD(0);
        scan_msg->angle_max = DEG2RAD(180);
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;
        scan_msg->angle_increment = horizontal_angle_resolution;
        uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment) * 2;
        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());

        if (time_synchronization_) {
            scan_msg->header.stamp = packet_timeStamp;
        } else {
            scan_msg->header.stamp = ros::Time::now();
        }
 	if (channel_num / 4 % 2 == 0){
		channel_num1 = channel_num;
		channel_num2 = channel_num + 4;
	}else{
		channel_num1 = channel_num;
		channel_num2 = channel_num - 4;
	}
        if (sweep_data->points.size() > 0) {
            for (size_t j = 0; j < sweep_data->points.size() - 1; ++j) {
                if (channel_num1 == sweep_data->points[j].line || channel_num2 == sweep_data->points[j].line) {
                    float horizontal_angle = sweep_data->points[j].azimuth;
                    uint point_idx = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_idx = (point_idx <= point_size) ? point_idx : (point_idx % point_size);
                    scan_msg->ranges[point_idx] = sweep_data->points[j].distance;
                    scan_msg->intensities[point_idx] = sweep_data->points[j].intensity;
                }
            }
            laserscan_pub.publish(scan_msg);
        }
        return;
    }

    void LslidarChDecoder::decodePacket(const RawPacket *packet) {

        // Compute the values for each firing
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {

            firings[point_idx].vertical_line = packet->points[point_idx].vertical_line;
            TwoBytes point_amuzith;
            point_amuzith.bytes[0] = packet->points[point_idx].azimuth_2;
            point_amuzith.bytes[1] = packet->points[point_idx].azimuth_1;
            firings[point_idx].azimuth = static_cast<double>(point_amuzith.value) * 0.01;
            ThreeBytes point_distance;
            point_distance.bytes[0] = packet->points[point_idx].distance1_3;
            point_distance.bytes[1] = packet->points[point_idx].distance1_2;
            point_distance.bytes[2] = packet->points[point_idx].distance1_1;
            firings[point_idx].distance = static_cast<double>(point_distance.value) * DISTANCE_RESOLUTION;
            firings[point_idx].intensity = packet->points[point_idx].intensity1;
        }
        return;
    }


    int LslidarChDecoder::convertCoordinate(const Firing lidardata) {
        if (!isPointInRange(lidardata.distance)) {
            return -1;
        }

        double x = 0.0, y = 0.0, z = 0.0;
        double cos_xita;
        // 垂直角度
        double sin_theat;
        double cos_theat;
        double _R_;
        // 水平角度
        double cos_H_xita;
        double sin_H_xita;
        double cos_xita_F;
        double sin_xita_F;

        int line_num = lidardata.vertical_line;
        if (line_num / 4 % 2 == 0) {
            cos_xita = cos((lidardata.azimuth / 2.0 + 22.5) * M_PI / 180);
        }else{
           cos_xita = cos((-lidardata.azimuth / 2.0 + 112.5) * M_PI / 180);
        }
        _R_ = theat2_c[line_num] * theat1_c[line_num] * cos_xita - theat2_s[line_num] * theat1_s[line_num];
        sin_theat = theat1_s[line_num] + 2 * _R_ * theat2_s[line_num];
        cos_theat = sqrt(1 - pow(sin_theat, 2));
        cos_H_xita = (2 * _R_ * theat2_c[line_num] * cos_xita - theat1_c[line_num]) / cos_theat;
        sin_H_xita = sqrt(1 - pow(cos_H_xita, 2));

        if(line_num / 4 % 2 == 0){
            cos_xita_F = (cos_H_xita + sin_H_xita) * sqrt(0.5);
            sin_xita_F = sqrt(1 - pow(cos_xita_F, 2));
        } else{
            cos_xita_F = (cos_H_xita + sin_H_xita) * (-sqrt(0.5));
            sin_xita_F = sqrt(1 - pow(cos_xita_F, 2));
        }

        x = lidardata.distance * cos_theat * cos_xita_F;
        y = lidardata.distance * cos_theat * sin_xita_F;
        z = lidardata.distance * sin_theat;

        sweep_data->points.push_back(lslidar_ch64w_msgs::LslidarChPoint());
        lslidar_ch64w_msgs::LslidarChPoint &new_point =        // new_point 为push_back最后一个的引用
                sweep_data->points[sweep_data->points.size() - 1];
        // Pack the data into point msg
        new_point.time = lidardata.time;
        new_point.x = x;
        new_point.y = y;
        new_point.z = z;
        new_point.azimuth = lidardata.azimuth * M_PI / 180;
        new_point.distance = lidardata.distance;
        new_point.intensity = lidardata.intensity;
        new_point.line = lidardata.vertical_line;
        return 0;
    }


    void LslidarChDecoder::packetCallback(
            const lslidar_ch64w_msgs::LslidarChPacketConstPtr &msg) {

        struct Firing lidardata;
        packet_timeStamp = msg->header.stamp;
        packet_end_time = packet_timeStamp.toSec();
        bool packetType = false;

        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx += 7) {
            if (msg->data[point_idx] == 0xff && msg->data[point_idx + 1] == 0xaa && msg->data[point_idx + 2] == 0xbb) {
                packetType = true;
            }
            double point_time = packet_end_time - point_time_diff * ((POINTS_PER_PACKET - point_idx) / 7 - 1);
            if (msg->data[point_idx] < 255) {
                memset(&lidardata, 0, sizeof(lidardata));
                lidardata.vertical_line = msg->data[point_idx];
                lidardata.azimuth = (msg->data[point_idx + 1] * 256 + msg->data[point_idx + 2]) * 0.01f;
                lidardata.distance = (msg->data[point_idx + 3] * 65536 + msg->data[point_idx + 4] * 256 +
                                      msg->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                lidardata.intensity = msg->data[point_idx + 6];
                lidardata.time = point_time;
                convertCoordinate(lidardata);
            }
            if (packetType) {
                //printf("---------------onesweep--------------------------\n");
                if (publish_point_cloud) { publishPointCloud(); }
                if (publish_laserscan) { publishLaserScan(); }
                packetType = false;
                sweep_data = lslidar_ch64w_msgs::LslidarChScanPtr(
                        new lslidar_ch64w_msgs::LslidarChScan());
            }
        }
        return;
    }

} // namespace lslidar_ch_decoder

