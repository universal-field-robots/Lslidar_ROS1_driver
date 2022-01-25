/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "lslidar_c16_driver/lslidar_c16_driver.h"
#include <lslidar_c16_msgs/LslidarC16ScanUnified.h>
#include <lslidar_c16_driver/lslidar_c16_driver.h>


namespace lslidar_c16_driver {
    static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;
    static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

    lslidarDriver::lslidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh) {
        scan_fill = false;
        // use private node handle to get parameters
        private_nh.param("frame_id", config_.frame_id, std::string("lslidar"));

        std::string tf_prefix = tf::getPrefixParam(private_nh);
        ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
        config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

        // get model name, validate string, determine packet rate
        private_nh.param("model", config_.model, std::string("LSC16"));
        double packet_rate;  // packet frequency (Hz)

        packet_rate = 840;   //20000/24

        private_nh.param("rpm", config_.rpm, 300.0);
        private_nh.param("return_mode", config_.return_mode, 1);
        double frequency = (config_.rpm / 60.0);  // expected Hz rate
        printf("driver return mode = %d\n", config_.return_mode);
        // default number of packets for each scan is a single revolution
        // (fractions rounded up)
        int npackets = (int) ceil(packet_rate / frequency);
        private_nh.param("npackets", config_.npackets, npackets);
        ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

        std::string dump_file;
        private_nh.param("pcap", dump_file, std::string(""));

        int msop_udp_port;
        private_nh.param("msop_port", msop_udp_port, (int) MSOP_DATA_PORT_NUMBER);
        int difop_udp_port;
        private_nh.param("difop_port", difop_udp_port, (int) DIFOP_DATA_PORT_NUMBER);

        scan_start = lslidar_c16_msgs::LslidarC16ScanUnifiedPtr(new lslidar_c16_msgs::LslidarC16ScanUnified);
        scan_start->packets.resize(10);

        // open rslidar input device or file
        if (dump_file != "")  // have PCAP file?
        {
            // read data from packet capture file
            msop_input_.reset(new lslidar_c16_driver::InputPCAP(private_nh, msop_udp_port, packet_rate, dump_file));
            difop_input_.reset(new lslidar_c16_driver::InputPCAP(private_nh, difop_udp_port, packet_rate, dump_file));
        } else {
            // read data from live socket
            msop_input_.reset(new lslidar_c16_driver::InputSocket(private_nh, msop_udp_port));
            difop_input_.reset(new lslidar_c16_driver::InputSocket(private_nh, difop_udp_port));

        }

        // raw packet output topic
        std::string output_packets_topic;
        private_nh.param("output_packets_topic", output_packets_topic, std::string("lslidar_packet"));
        msop_output_ = node.advertise<lslidar_c16_msgs::LslidarC16ScanUnified>(output_packets_topic, 10);

        std::string output_difop_topic;
        private_nh.param("output_difop_topic", output_difop_topic, std::string("lslidar_packet_difop"));
        difop_output_ = node.advertise<lslidar_c16_msgs::LslidarC16DifPacket>(output_difop_topic, 10);

        difop_thread_ = boost::shared_ptr<boost::thread>(
                new boost::thread(boost::bind(&lslidarDriver::difopPoll, this)));
        private_nh.param("time_synchronization", time_synchronization_, false);


        if (time_synchronization_) {
            output_sync_ = node.advertise<sensor_msgs::TimeReference>("sync_header", 1);
        }
    }


    lslidarDriver::~lslidarDriver() {
        if (difop_thread_ != NULL) {
            printf("error");
            difop_thread_->interrupt();
            difop_thread_->join();
        }

    }

/** poll the device
 *
 *  @returns true unless end of file reached
 */
    bool lslidarDriver::poll(void) {  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_c16_msgs::LslidarC16ScanUnifiedPtr scan(new lslidar_c16_msgs::LslidarC16ScanUnified);

        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        int mode = config_.return_mode;

        if (difop_input_->getUpdateFlag()) {
            int packets_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND / BLOCKS_ONE_CHANNEL_PER_PKT);
            packets_rate = ceil(packets_rate / 2);
            config_.rpm = difop_input_->getRpm();

//    if(config_.rpm >= 300 && config_.rpm < 600){
//      config_.rpm = 300;
//    }else if(config_.rpm >= 600 && config_.rpm < 1200){
//      config_.rpm = 600;
//    }else if(config_.rpm >= 1200){
//      config_.rpm = 1200;
//    }
            //config_.npackets = ceil(packets_rate * 60 / config_.rpm) * mode;
            config_.npackets = ceil(packets_rate * 60 / config_.rpm) * mode;

            config_.npackets = config_.npackets * 11 / 10;

            difop_input_->clearUpdateFlag();
            //ROS_INFO("packet rate is %d, rpm is %3.3f, npacket is %d", packets_rate, config_.rpm, config_.npackets);
        }

        //ROS_INFO("rpm is %3.3f, npacket is %d", config_.rpm, config_.npackets);
        scan->packets.clear();
        scan->packets.resize(config_.npackets);
        int azi1, azi2;
        if (scan_fill) {
            scan->packets[0] = scan_start->packets[0];
        } else {
            while (true) {
                while (true) {
                    int rc = msop_input_->getMsPacket(&scan->packets[0], config_.time_offset);
                    if (rc == 0)
                        break;
                    if (rc < 0)
                        return false;
                }

                azi1 = 256 * scan->packets[0].data[3] + scan->packets[0].data[2];
                azi2 = 256 * scan->packets[0].data[1103] + scan->packets[0].data[1102];
                if (azi1 > 35000 && azi2 < 1000) break;
            }
        }
        scan_fill = false;
        // use in standard behaviour only

        for (int i = 1; i < config_.npackets; ++i) {

            while (true) {
                // keep reading until full packet received
                //ROS_INFO_STREAM("time_offset: " << config_.time_offset);
                int rc = msop_input_->getMsPacket(&scan->packets[i], config_.time_offset);
                if (rc == 0)
                    break;  // got a full packet?
                if (rc < 0)
                    return false;  // end of file reached?

            }
            azi1 = 256 * scan->packets[i].data[3] + scan->packets[i].data[2];
            azi2 = 256 * scan->packets[i].data[1103] + scan->packets[i].data[1102];
            //azi2 = (azi2 +20) % 36000;
            //if ( (azi1 > 35800 && azi2 < 100 )) {

            if ((azi1 > 35000 && azi2 < 1000) || (azi1 < 500 && i > config_.npackets / 2)) {

                scan_fill = true;
                scan_start->packets[0] = scan->packets[i];
                // ROS_INFO_STREAM("azi1: " << azi1 <<"  "<< "azi2: " << azi2 << "   i:" << i);
                break;
            }
        }

        if (time_synchronization_) {
            sensor_msgs::TimeReference sync_header;

            // it is already the msop msg
            // use the first packets
            lslidar_c16_msgs::LslidarC16MsPacket pkt = scan->packets[0];
            uint64_t packet_timestamp;

            struct tm cur_time;
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_year = pkt.data[1200] + 2000 - 1900;
            cur_time.tm_mon = pkt.data[1201] - 1;
            cur_time.tm_mday = pkt.data[1202];
            cur_time.tm_hour = pkt.data[1203] + 8;
            cur_time.tm_min =pkt.data[1204];
            cur_time.tm_sec = pkt.data[1205] + 1;
            this->pointcloudTimeStamp = mktime(&cur_time);
            GPSCurrentTS = this->pointcloudTimeStamp;

            packet_timestamp = (pkt.data[1206] +
                                pkt.data[1207] * pow(2, 8) +
                                pkt.data[1208] * pow(2, 16) +
                                pkt.data[1209] * pow(2, 24)) * 1e3; //ns

            timeStamp = ros::Time(GPSCurrentTS, packet_timestamp);// s,ns


            //ROS_INFO("Lidar_time: %f, GPS_time:%lu, fpga_time: ns:%lu",timeStamp.toSec(), GPSCurrentTS, packet_timestamp);

            scan->header.stamp = timeStamp;
            sync_header.header.stamp = timeStamp;
            output_sync_.publish(sync_header);
        }

        // publish message using time of last packet read
        //  ROS_INFO("Publishing a full scan.");
        else {
            scan->header.stamp = scan->packets.back().stamp;
		//scan->header.stamp = ros::Time::now();
        }
        scan->header.frame_id = config_.frame_id;
        msop_output_.publish(scan);

        return true;
    }

    void lslidarDriver::difopPoll(void) {
        // reading and publishing scans as fast as possible.
        lslidar_c16_msgs::LslidarC16DifPacketPtr difop_packet_ptr(new lslidar_c16_msgs::LslidarC16DifPacket);
        while (ros::ok()) {
            // keep reading
            lslidar_c16_msgs::LslidarC16DifPacket difop_packet_msg;
            int rc = difop_input_->getDifPacket(&difop_packet_msg, config_.time_offset);
            if (rc == 0) {
                //std::cout << "Publishing a difop data." << std::endl;
                ROS_DEBUG("Publishing a difop data.");
                *difop_packet_ptr = difop_packet_msg;
                difop_output_.publish(difop_packet_ptr);
            }
            else if (rc < 0)
                return;  // end of file reached?
            ros::spinOnce();
        }
    }

// add for time synchronization
}  // namespace lslidar_c16_driver
