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

#ifndef LSLIDAR_Ch_DRIVER_H
#define LSLIDAR_Ch_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <lslidar_ch_msgs/LslidarChPacket.h>
#include <lslidar_ch_msgs/LslidarChScanUnified.h>

namespace lslidar_ch_driver {
static uint16_t PACKET_SIZE = 1206;
class LslidarChDriver {
public:

    LslidarChDriver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarChDriver();
    bool initialize();
    bool polling();
    void initTimeStamp(void);
    void getFPGA_GPSTimeStamp(lslidar_ch_msgs::LslidarChPacketPtr &packet);
    typedef boost::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
    typedef boost::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_ch_msgs::LslidarChPacketPtr& msg);

    // Ethernet relate variables
    std::string lidar_ip_string;
    std::string group_ip_string;
    std::string frame_id;

    in_addr lidar_ip;
    int UDP_PORT_NUMBER;
    int socket_id;
    int cnt_gps_ts;
    bool use_gps_;
    bool add_multicast;

    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Publisher packet_pub;    

    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

    uint64_t pointcloudTimeStamp;
    uint64_t GPSStableTS;
    uint64_t GPSCountingTS;
    uint64_t last_FPGA_ts;
    uint64_t GPS_ts;
    unsigned char packetTimeStamp[10];
    struct tm cur_time;
    unsigned short int us;
    unsigned short int ms;
    ros::Time timeStamp;
};

typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;

} // namespace lslidar_driver

#endif // _LSLIDAR_Ch_DRIVER_H_
