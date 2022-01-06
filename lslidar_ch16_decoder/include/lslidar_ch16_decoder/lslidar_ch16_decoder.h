/*
 * This file is part of lslidar_ch16 driver.
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

#ifndef LSLIDAR_Ch_DECODER_H
#define LSLIDAR_Ch_DECODER_H

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lslidar_ch16_msgs/LslidarChPacket.h>
#include <lslidar_ch16_msgs/LslidarChPoint.h>
#include <lslidar_ch16_msgs/LslidarChScan.h>
#include <lslidar_ch16_msgs/LslidarChSweep.h>
#include <lslidar_ch16_msgs/LslidarChLayer.h>


namespace lslidar_ch16_decoder {

// Raw lslidar packet constants and structures.
static const int SIZE_BLOCK      = 100;
static const int RAW_SCAN_SIZE   = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE =
        (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// valid packets with readings up to 200.0.
static const double DISTANCE_MAX        = 200.0;        /**< meters */
static const double DISTANCE_RESOLUTION = 0.01; /**< meters */
static const double DISTANCE_MAX_UNITS  =
        (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
//static const uint8_t UPPER_BANK_ONE = 0xff;   //flag 0
//static const uint16_t UPPER_BANK_TWO =0xbbaa;    //flag 1
//static const uint16_t UPPER_BANK_THREE=0xddcc;    //flag 2
//static const uint8_t UPPER_BANK_FOUR = 0xee;  //flag 3

/** Special Defines for LSCh support **/
static const int PACKET_SIZE        = 1206;    
static const int POINTS_PER_PACKET  = 171;

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_laser_altitude[4] = {
        -0.06981317007977318,-0.0466002910282486,
        -0.023387411976724018,-0.0,
};

static const double sin_scan_laser_altitude[4] = {
    std::sin(scan_laser_altitude[0]), std::sin(scan_laser_altitude[1]),
    std::sin(scan_laser_altitude[2]), std::sin(scan_laser_altitude[3]),
};

static const double scan_mirror_altitude[4] = {
        -0.0,0.005759586531581287,
        0.011693705988362009,0.017453292519943295,
};

static const double sin_scan_mirror_altitude[4] = {
    std::sin(scan_mirror_altitude[0]), std::sin(scan_mirror_altitude[1]),
    std::sin(scan_mirror_altitude[2]), std::sin(scan_mirror_altitude[3]),
};

typedef struct{
    double distance;
    double intensity;
}point_struct;

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

class LslidarChDecoder {
public:
    LslidarChDecoder(ros::NodeHandle& n, ros::NodeHandle& pn);
    LslidarChDecoder(const LslidarChDecoder&) = delete;
    LslidarChDecoder operator=(const LslidarChDecoder&) = delete;
    ~LslidarChDecoder() {return;}

    bool initialize();

    typedef boost::shared_ptr<LslidarChDecoder> LslidarChDecoderPtr;
    typedef boost::shared_ptr<const LslidarChDecoder> LslidarChDecoderConstPtr;

private:

    union TwoBytes {
        uint16_t value;
        uint8_t  bytes[2];
    };
    
    union ThreeBytes {
        uint32_t value;
        uint8_t  bytes[3];
    };

    struct Point {
        uint8_t vertical_line;        //0-127 
        uint8_t azimuth_1;
        uint8_t azimuth_2;      ///< 1500-16500, divide by 100 to get degrees
        uint8_t distance_1;
        uint8_t distance_2;
        uint8_t distance_3;
        uint8_t intensity;
    };
    
    struct RawPacket {
        Point points[POINTS_PER_PACKET];
        uint32_t time_stamp;
        uint8_t factory[2];
    };

    struct Firing {
        //double vertical_angle;
        int vertical_line;
        double azimuth;
        double distance;
        int intensity;
    };

    // Intialization sequence
    bool loadParameters();
    bool createRosIO();


    // Callback function for a single lslidar packet.
    int checkPacketValidity(const RawPacket* packet);
    void decodePacket(const RawPacket* packet);
    void packetCallback(const lslidar_ch16_msgs::LslidarChPacketConstPtr& msg);
    // Publish data
    void publishPointCloud();
    void publishLaserScan();
    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        // According to the user manual,
        return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
		//return static_cast<double>(raw_azimuth) / 100.0;
    }
     double verticalLineToAngle(const uint16_t& vertical_line) {
        // According to the user manual,
       // return static_cast<double>(vertical_line)*0.25-17;
                return (static_cast<double>(vertical_line)*0.33-6.67) * DEG_TO_RAD;
               // return (static_cast<double>(vertical_line)*0.25-17) * DEG_TO_RAD;
    }
    
    // configuration degree base
    int point_num;
    double angle_base;
    
    // Configuration parameters
    double min_range;
    double max_range;
    int frequency;
    bool publish_point_cloud;
    bool publish_laserscan;
    bool time_synchronization;
    double horizontal_angle_resolution;
    ros::Time packet_timestamp;
    double packet_end_time;

    double last_azimuth;
    double packet_start_time;
    int layer_num;
    Firing firings[POINTS_PER_PACKET];
    // ROS related parameters
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    //std::string fixed_frame_id;
    std::string frame_id;
    int channel_num;
   
    sensor_msgs::PointCloud2 point_cloud_data;
    lslidar_ch16_msgs::LslidarChScanPtr sweep_data;

    ros::Subscriber packet_sub;
    ros::Publisher point_cloud_pub;
    ros::Publisher laserscan_pub;
};

typedef LslidarChDecoder::LslidarChDecoderPtr LslidarChDecoderPtr;
typedef LslidarChDecoder::LslidarChDecoderConstPtr LslidarChDecoderConstPtr;
typedef PointXYZIRT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

} // end namespace lslidar_ch16_decoder


POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch16_decoder::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity,intensity)
                                          (uint16_t,ring,ring)
                                      (double, time, time))
#endif
