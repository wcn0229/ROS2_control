#ifndef DS_node_TALKER_HPP
#define DS_node_TALKER_HPP

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#ifdef ENABLED_GEO_POSE_STAMPED
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#endif

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ds_convert.h"

class DsnodeTalker : public rclcpp::Node
{
public:
    DsnodeTalker();
    ~DsnodeTalker();
	
    static const int MaxMsgLen = 2048;

private:	
    void init();

    void execute();
    

    rclcpp::Publisher<ds_node::msg::Kalman>::SharedPtr kalman_pub_;
    rclcpp::Publisher<ds_node::msg::GnssHmr>::SharedPtr gnssHmr_pub_;
    rclcpp::Publisher<ds_node::msg::RawIMU>::SharedPtr RawIMU_pub_;
    rclcpp::Publisher<ds_node::msg::SolutionStatus>::SharedPtr SolutionStatus_pub_;
    rclcpp::Publisher<ds_node::msg::CompactNav>::SharedPtr compactNav_pub_;
#ifdef ENABLED_GEO_POSE_STAMPED    
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_pub_;
#endif
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navfix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr attitude_imu_pub_;
    rclcpp::Publisher<ds_node::msg::EulerAttitude>::SharedPtr EulerAttitude_pub_;
    rclcpp::Publisher<ds_node::msg::TimeSync>::SharedPtr timeSync_pub_;
    rclcpp::Publisher<ds_node::msg::Geoid>::SharedPtr geoid_pub_;
    rclcpp::Publisher<ds_node::msg::CorrectedIMU>::SharedPtr CorrectedIMU_pub_;
    rclcpp::Publisher<ds_node::msg::LeapSeconds>::SharedPtr leapSeconds_pub_;
    rclcpp::Publisher<ds_node::msg::NmeaGGA>::SharedPtr nmeaGGA_pub_;
    rclcpp::Publisher<ds_node::msg::NmeaGGA>::SharedPtr nmeaGGA2_pub_;
    rclcpp::Publisher<ds_node::msg::Dmi>::SharedPtr dmi_pub_;
    
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
	
	enum DecodeStatus
    {
       _SYNC = 0,
       _HEAD,
       _MSG
    };
	
    DecodeStatus decode_status_ = _SYNC;
	
    uint8_t buf[MaxMsgLen];
    int bufpos = 0;
    int msglen = 0;

    void getTimeStamp(const double t, builtin_interfaces::msg::Time& stamp);
    void parseAttitudeImu(uint8_t* buf, sensor_msgs::msg::Imu& imu);
    void parseCompactNav(
       uint8_t*          buf, 
       ds_node::msg::CompactNav& msg,
       ds::ref_frame_trans_type  frame_trans);
};

#endif