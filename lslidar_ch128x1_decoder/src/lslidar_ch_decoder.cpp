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
            packet_end_time(0.0),
            sweep_data(new lslidar_ch128x1_msgs::LslidarChScan()) {
        packet_timeStamp.sec = 0;
        packet_timeStamp.nsec = 0;

        return;
    }

    bool LslidarChDecoder::loadParameters() {
        pnh.param<double>("min_range", min_range, 0.5);
        pnh.param<double>("max_range", max_range, 100.0);
        pnh.param<int>("frequency", frequency, 10);
        pnh.param<string>("pointcloud_topic", pointcloud_topic, "lslidar_point_cloud");
        pnh.param<bool>("publish_point_cloud", publish_point_cloud, true);
        pnh.param<bool>("publish_laserscan", publish_laserscan, false);
        pnh.param<int>("channel_num", channel_num, 8);
        //pnh.param<string>("fixed_frame_id", fixed_frame_id, "map");
        pnh.param<string>("frame_id", frame_id, "lslidar");
        pnh.param<bool>("time_synchronization", time_synchronization, false);
        ROS_INFO("Using GPS timestamp or not %d", time_synchronization);
        if (publish_laserscan) {
            if (channel_num < 0) {
                channel_num = 0;
                ROS_WARN("channel_num outside of the index, select channel 0 instead!");
            } else if (channel_num > 127) {
                channel_num = 127;
                ROS_WARN("channel_num outside of the index, select channel 15 instead!");
            }
            ROS_INFO("select channel num: %d", channel_num);
        }

        switch (frequency) {
            case 5:
                horizontal_angle_resolution = DEG2RAD(0.1);
                break;
            case 20:
                horizontal_angle_resolution = DEG2RAD(0.4);
                break;
            default:
                horizontal_angle_resolution = DEG2RAD(0.2);
        }

        return true;
    }

    bool LslidarChDecoder::createRosIO() {
        packet_sub = nh.subscribe<lslidar_ch128x1_msgs::LslidarChPacket>(
                "lslidar_packet_ch128x1", 100, &LslidarChDecoder::packetCallback, this);
        point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
                pointcloud_topic, 10);
        laserscan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
        return true;
    }

    bool LslidarChDecoder::initialize() {
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
        VPoint point;
        if (sweep_data->points.size() > 0) {
            if (time_synchronization) {
                point_cloud->header.stamp = pcl_conversions::toPCL(packet_timeStamp);
            } else {
                point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
            }

            for (j = 1; j < sweep_data->points.size() - 1; ++j) {
                point.timestamp = sweep_data->points[j].time;
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

        sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
        scan_msg->header.frame_id = frame_id;
        if (time_synchronization) {
            scan_msg->header.stamp = packet_timeStamp;
        } else {
            scan_msg->header.stamp = ros::Time::now();
        }
        scan_msg->angle_min = DEG2RAD(30);
        scan_msg->angle_max = DEG2RAD(150);
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;
        scan_msg->angle_increment = horizontal_angle_resolution;
        uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        if (sweep_data->points.size() > 0) {
            for (size_t j = 0; j < sweep_data->points.size() - 1; ++j) {
                if (channel_num == sweep_data->points[j].line) {
                    float horizontal_angle = sweep_data->points[j].azimuth;
                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index <= point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_index] = sweep_data->points[j].distance;
                    scan_msg->intensities[point_index] = sweep_data->points[j].intensity;
                }

            }
            laserscan_pub.publish(scan_msg);
        }
    }


    int LslidarChDecoder::convertCoordinate(struct Firing lidardata) {
        if (!isPointInRange(lidardata.distance)) {
            return -1;
        }

        double x = 0.0, y = 0.0, z = 0.0;
        double sin_theat;
        double cos_theat;
        //中间变量
        double _R_ = cos_theta_2[lidardata.vertical_line] * cos_theta_1[lidardata.vertical_line] *
                     cos(lidardata.azimuth / 2.0) -
                     sin_theta_2[lidardata.vertical_line] * sin_theta_1[lidardata.vertical_line];
        sin_theat = sin_theta_1[lidardata.vertical_line] + 2 * _R_ * sin_theta_2[lidardata.vertical_line];
        cos_theat = sqrt(1 - pow(sin_theat, 2));

        x = lidardata.distance * cos_theat * cos(lidardata.azimuth);
        y = lidardata.distance * cos_theat * sin(lidardata.azimuth);
        z = lidardata.distance * sin_theat;

        double x_coord = x;
        double y_coord = y;
        double z_coord = z;
        sweep_data->points.push_back(lslidar_ch128x1_msgs::LslidarChPoint());
        lslidar_ch128x1_msgs::LslidarChPoint &new_point =        // new_point 为push_back最后一个的引用
                sweep_data->points[sweep_data->points.size() - 1];

        new_point.x = x_coord;
        new_point.y = y_coord;
        new_point.z = z_coord;
        new_point.vertical_angle = RAD2DEG(asin(sin_theat));
        new_point.azimuth = lidardata.azimuth;
        new_point.distance = lidardata.distance;
        new_point.intensity = lidardata.intensity;
        new_point.line = lidardata.vertical_line;
        new_point.time = lidardata.time;

        return 0;

    }


    void LslidarChDecoder::packetCallback(
            const lslidar_ch128x1_msgs::LslidarChPacketConstPtr &msg) {
        struct Firing lidardata;


        // Convert the msg to the raw packet type.

        packet_timeStamp = msg->header.stamp;
        packet_end_time = packet_timeStamp.toSec();
        bool packetType = false;

        for (int i = 0; i < 128; i++) {
            sin_theta_1[i] = sin(big_angle[i / 4] * M_PI / 180);
            cos_theta_1[i] = cos(big_angle[i / 4] * M_PI / 180);

            // sinTheta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
            if (abs(msg->prism_angle[0]) < 1e-6 && abs(msg->prism_angle[1]) < 1e-6 && abs(msg->prism_angle[2]) < 1e-6 &&
                abs(msg->prism_angle[3]) < 1e-6) {
                sin_theta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
                cos_theta_2[i] = cos((i % 4) * (-0.17) * M_PI / 180);
            } else {
                sin_theta_2[i] = sin(msg->prism_angle[i % 4] * M_PI / 180);
                cos_theta_2[i] = cos(msg->prism_angle[i % 4] * M_PI / 180);
            }
        }
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx += 7) {
            if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) &&
                (msg->data[point_idx + 2] == 0xbb)) {
                packetType = true;
            }
            // Compute the time of the point
            double point_time = packet_end_time - 0.868 * ((POINTS_PER_PACKET - point_idx) / 7 - 1) * 1e-6;
            if (msg->data[point_idx] < 255) {
                memset(&lidardata, 0, sizeof(lidardata));
                lidardata.vertical_line = msg->data[point_idx];
                lidardata.azimuth =
                        (msg->data[point_idx + 1] * 256 + msg->data[point_idx + 2]) / 100.f * DEG_TO_RAD;
                lidardata.distance = (msg->data[point_idx + 3] * 65536 + msg->data[point_idx + 4] * 256 +
                                      msg->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                lidardata.intensity = msg->data[point_idx + 6];
                lidardata.time = point_time;
                convertCoordinate(lidardata);
            }
            if (packetType) {
                //("---------------onesweep--------------------------\n");
                if (publish_point_cloud) { publishPointCloud(); }
                if (publish_laserscan) { publishLaserScan(); }
                packetType = false;
                sweep_data = lslidar_ch128x1_msgs::LslidarChScanPtr(
                        new lslidar_ch128x1_msgs::LslidarChScan());
            }
        }

        return;
    }

} // end namespace lslidar_ch_decoder

