/*
 * This file is part of lslidar_m10 driver.
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

#include <lslidar_m10_decoder/lslidar_m10_decoder.h>

using namespace std;

namespace lslidar_m10_decoder {
LslidarM10Decoder::LslidarM10Decoder(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
    last_degree(0.0),
	scan_size(16),
	idx(0){
	return;
	}

bool LslidarM10Decoder::loadParameters() {
    pnh.param<int>("point_num", point_num, 1000);
    pnh.param<double>("min_range", min_range, 0.3);
    pnh.param<double>("max_range", max_range, 100.0);
	pnh.param("angle_disable_min", angle_disable_min,0.0);
	pnh.param("angle_disable_max", angle_disable_max, 0.0);
    pnh.param<string>("child_frame_id", child_frame_id, "laser_link");
	pnh.param<string>("scan_topic", scan_topic_, "scan");
	
    angle_base = 360.0 / point_num;

	scan_points_.resize(point_num+100);
    return true;
}

bool LslidarM10Decoder::createRosIO() {
	difop_sub_ = nh.subscribe("lslidar_packet_difop", 10, &LslidarM10Decoder::processDifop, (LslidarM10Decoder*)this);
    packet_sub = nh.subscribe<lslidar_m10_msgs::LslidarM10Packet>("lslidar_packet", 100, &LslidarM10Decoder::packetCallback, this, ros::TransportHints().tcpNoDelay(true));
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic_, 100);
	device_pub = nh.advertise<lslidar_m10_msgs::LslidarM10Difop>("difop_information", 100);

    return true;
}

bool LslidarM10Decoder::initialize() {
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

void LslidarM10Decoder::processDifop(const lslidar_m10_msgs::LslidarM10Packet::ConstPtr& difop_msg)
{
	  const uint8_t* data = &difop_msg->data[0];
	  
	  Difop_data = lslidar_m10_msgs::LslidarM10DifopPtr(
							new lslidar_m10_msgs::LslidarM10Difop());
							
	  // check header
	  if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a)
	  {
		return;
	  }

	 Difop_data->rpm = (data[20]<<8) | data[21];
	 //Difop_data->temperature = (data[28]*256 + data[29]) * 330 / 4096 - 50;
	 
	 device_pub.publish(Difop_data);
}

void LslidarM10Decoder::publishScan()
{
    sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
	
    scan->header.frame_id = child_frame_id;
    scan->header.stamp = ros::Time::now();  // timestamp will obtained from sweep data stamp
	
    //scan->time_increment = 0.05;
    //scan->scan_time= 0.1;
    scan->angle_min = 0.0;
    scan->angle_max = 2.0*M_PI;
    scan->angle_increment = (scan->angle_max - scan->angle_min)/point_num;
	
    scan->range_min = min_range;
    scan->range_max = max_range;
    scan->ranges.reserve(point_num);
    scan->ranges.assign(point_num, std::numeric_limits<float>::infinity());
    scan->intensities.reserve(point_num);
    scan->intensities.assign(point_num, std::numeric_limits<float>::infinity());
	
	for(uint16_t i = 0; i < scan_points_.size(); i++)
	{
		int point_idx = scan_points_[i].degree / angle_base;

		if (point_idx >= point_num)
		  point_idx = point_idx - point_num;
		if (point_idx < 0)
		  point_idx = point_idx + point_num;

		scan->ranges[point_num - point_idx] = scan_points_[i].range;
		scan->intensities[point_num - point_idx] = 0;
	}
	
    scan_pub.publish(scan);
}

void LslidarM10Decoder::packetCallback(
        const lslidar_m10_msgs::LslidarM10PacketConstPtr& msg) {

	 Difop_data = lslidar_m10_msgs::LslidarM10DifopPtr(new lslidar_m10_msgs::LslidarM10Difop());
	
	if (msg->data[0] != 0xA5 || msg->data[1] != 0x5A) return;
	
	TwoBytes raw_degree;
	raw_degree.bytes[0] = msg->data[3];
	raw_degree.bytes[1] = msg->data[2];
	degree = raw_degree.distance/100.f;
	
	raw_degree.bytes[0] = msg->data[5];
	raw_degree.bytes[1] = msg->data[4];
	Difop_data->rpm = 2500000/raw_degree.distance;
	
	device_pub.publish(Difop_data);
	 
	TwoBytes raw_range;
	int invalidValue = 0;

	for (size_t num = 2; num < 86; num+=2)
	{
		raw_range.bytes[0] = msg->data[num + 5];
		raw_range.bytes[1] = msg->data[num + 4];
		
		if (raw_range.distance != 65535)
		{
			scan_points_[idx].range = double(raw_range.distance) / 1000.f;
			scan_points_[idx].intensity = 0;
			idx++;
		}
		else
		{
			invalidValue++;
		}
	}

	invalidValue = 42 - invalidValue;

	for (size_t i = 0; i < invalidValue; i++)
	{
		if ((degree + (15.0 / invalidValue * i)) > 360.0)
			scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i) - 360.0;
		else
			scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i);
	}

	if (degree < last_degree) 	
	{

		idx = 0;
		for(int k=0;k<scan_points_.size();k++)
		{
			if(angle_disable_min < angle_disable_max)
			{
				if(scan_points_[k].degree > angle_disable_min && scan_points_[k].degree < angle_disable_max)
					scan_points_[k].range = 0;
			}
			else if(angle_disable_min != 0.0 || angle_disable_max!= 0.0)
			{
				if(scan_points_[k].degree > angle_disable_min || scan_points_[k].degree < angle_disable_max)
					scan_points_[k].range = 0;
			}
			
			if(scan_points_[k].range < min_range || scan_points_[k].range > max_range)
				scan_points_[k].range = 0;
		}

		publishScan();

		for(int k=0; k<scan_points_.size(); k++)
		{
			scan_points_[k].range = 0;
			scan_points_[k].degree = 0;
		}
	}
	last_degree = degree;
	
    return;
}

} // end namespace lslidar_m10_decoder

