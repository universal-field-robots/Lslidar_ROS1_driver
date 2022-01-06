#ifndef LSLIDAR_CH16_DRIVER_INPUT_H
#define LSLIDAR_CH16_DRIVER_INPUT_H

#include <stdio.h>
#include <unistd.h>
#include <pcap.h>
#include <ros/ros.h>
#include <sys/file.h>
#include <string>
#include <lslidar_ch16_msgs/LslidarChPacket.h>
#include <netinet/in.h>
#include <errno.h>
#include <signal.h>
#include <arpa/inet.h>
#include <poll.h>




namespace lslidar_ch_driver{
    static uint16_t PACKET_SIZE = 1206;
    static uint16_t  MSOP_DATA_PORT_NUMBER = 2368; //lslidar default data port on pc
    static uint16_t DIFOP_DATA_PORT_NUMBER = 2369; //lslidar default difop port on pc

class Input {
public:
    Input(ros::NodeHandle private_nh,uint16_t port);
    virtual ~Input(){}
    virtual int getPacket(lslidar_ch16_msgs::LslidarChPacketPtr& packet) = 0;

protected:
    std::string  devip_str_;
    int cur_rpm_;
    bool npkt_update_flag_;
    bool add_multicast;
    std::string group_ip;

};


class InoutSocket: public Input{
public:
    InoutSocket(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER);
    virtual ~InoutSocket();
    virtual int getPacket(lslidar_ch16_msgs::LslidarChPacketPtr& packet);

private:
    int sockfd_;
    in_addr devip_;

};


class InputPCAP : public Input{
public:
    InputPCAP(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0,
            std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);
    virtual ~InputPCAP();
    virtual int getPacket(lslidar_ch16_msgs::LslidarChPacketPtr& pkt);

private:
    ros::Rate packet_rate_;
    std::string filename_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;

};

}
#endif //LSLIDAR_CH16_DRIVER_INPUT_H
