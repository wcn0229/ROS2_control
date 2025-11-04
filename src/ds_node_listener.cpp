#include "rclcpp/rclcpp.hpp"

#include "ds_node/msg/kalman.hpp"
#include "ds_node/msg/raw_imu.hpp"
#include "ds_node/msg/solution_status.hpp"
#include "ds_node/msg/compact_nav.hpp"
#include "ds_node/msg/euler_attitude.hpp"
#include "ds_node/msg/time_sync.hpp"
#include "ds_node/msg/geoid.hpp"
#include "ds_node/msg/corrected_imu.hpp"

//  please don't include .h file, only .hpp allowed
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#ifdef ENABLED_GEO_POSE_STAMPED
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#endif

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h> /* Error number definitions */
#include <unistd.h>
#include <string.h>

#define log_file "/tmp/gpstime.log"
#define RAD_TO_DEG 57.295779513082323

// int fd_log;
FILE *flog = NULL;
// node pointer
rclcpp::Node::SharedPtr node = nullptr;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void dumpKalmanMessage(const ds_node::msg::Kalman::SharedPtr &kalmsg)
{
    if (flog)
        fprintf(flog, "%f\n", kalmsg->gps_time);

    RCLCPP_INFO(node->get_logger(), "SystemTime=%f", kalmsg->system_time);
    RCLCPP_INFO(node->get_logger(), "GPSTime=%f", kalmsg->gps_time);
    RCLCPP_INFO(node->get_logger(), "Latitude=%.9lf", kalmsg->latitude * RAD_TO_DEG);
    RCLCPP_INFO(node->get_logger(), "Longitude=%.9lf", kalmsg->longitude * RAD_TO_DEG);
    RCLCPP_INFO(node->get_logger(), "EllipsoidalHeight=%f", kalmsg->ellipsoidal_height);
    RCLCPP_INFO(node->get_logger(), "VelocityNorth=%f", kalmsg->velocity_north);
    RCLCPP_INFO(node->get_logger(), "VelocityEast=%f", kalmsg->velocity_east);
    RCLCPP_INFO(node->get_logger(), "VelocityDown=%f", kalmsg->velocity_down);
    RCLCPP_INFO(node->get_logger(), "Roll=%f", kalmsg->roll);
    RCLCPP_INFO(node->get_logger(), "Pitch=%f", kalmsg->pitch);
    RCLCPP_INFO(node->get_logger(), "Heading=%f", kalmsg->heading);
    RCLCPP_INFO(node->get_logger(), "PositionMode=%d", kalmsg->position_mode);
    RCLCPP_INFO(node->get_logger(), "VelocityMode=%d", kalmsg->velocity_mode);
    RCLCPP_INFO(node->get_logger(), "AttitudeStatus=%d\n", kalmsg->attitude_status);
}

void dumpRawIMUMessage(const ds_node::msg::RawIMU::SharedPtr &imsg)
{

    if (flog)
        fprintf(flog, "%lf\n", imsg->system_time);

    RCLCPP_INFO(node->get_logger(), "SystemTime=%f", imsg->system_time);
    RCLCPP_INFO(node->get_logger(), "Acceleration=[%f,%f,%f]", imsg->acceleration[0], imsg->acceleration[1], imsg->acceleration[2]);
    RCLCPP_INFO(node->get_logger(), "RotationRate=[%f,%f,%f]\n", imsg->rotation_rate[0], imsg->rotation_rate[1], imsg->rotation_rate[2]);
}

void dumpSolutionStatus(const ds_node::msg::SolutionStatus::SharedPtr &smsg)
{

    if (flog)
        fprintf(flog, "%lf\n", smsg->system_time);

    RCLCPP_INFO(node->get_logger(), "system_time=%f", smsg->system_time);
    RCLCPP_INFO(node->get_logger(), "GpsWeekNumber=%d", smsg->gps_week_number);
    RCLCPP_INFO(node->get_logger(), "NumberOfSVs=%d", smsg->number_of_svs);
    RCLCPP_INFO(node->get_logger(), "ProcessingMode=%d", smsg->processing_mode);
    RCLCPP_INFO(node->get_logger(), "GpsTimeWeek=%f", smsg->gps_time_week);
    RCLCPP_INFO(node->get_logger(), "PositionRMS=[%f,%f,%f]", smsg->position_rms[0], smsg->position_rms[1], smsg->position_rms[2]);
    RCLCPP_INFO(node->get_logger(), "VelocityRMS=[%f,%f,%f]", smsg->velocity_rms[0], smsg->velocity_rms[1], smsg->velocity_rms[2]);
    RCLCPP_INFO(node->get_logger(), "AttitudeRMS=[%f,%f,%f]\n", smsg->attitude_rms[0], smsg->attitude_rms[1], smsg->attitude_rms[2]);
}

void dumpICDMessage(const ds_node::msg::CompactNav::SharedPtr &msg)
{
    if (flog)
        fprintf(flog, "%lf\n", msg->gps_time_week);

    RCLCPP_INFO(node->get_logger(), "GpsTimeOfWeek=%.3lf", msg->gps_time_week);
    RCLCPP_INFO(node->get_logger(), "Latitude=%.9lf", msg->latitude * RAD_TO_DEG);
    RCLCPP_INFO(node->get_logger(), "Longitude=%.9lf", msg->longitude * RAD_TO_DEG);
    RCLCPP_INFO(node->get_logger(), "Altitude=%.3lf", msg->altitude);
    RCLCPP_INFO(node->get_logger(), "VelocityNED=[%.3lf,%.3lf,%.3lf]", msg->velocity_ned[0], msg->velocity_ned[1], msg->velocity_ned[2]);
    RCLCPP_INFO(node->get_logger(), "Attitude=[%e,%e,%e,%e]", msg->quaternion[0], msg->quaternion[1], msg->quaternion[2], msg->quaternion[3]);
    RCLCPP_INFO(node->get_logger(), "Acceleration=[%e,%e,%e]", msg->acceleration[0], msg->acceleration[1], msg->acceleration[2]);
    RCLCPP_INFO(node->get_logger(), "RotationRate=[%e,%e,%e]", msg->rotation_rate[0], msg->rotation_rate[1], msg->rotation_rate[2]);
    RCLCPP_INFO(node->get_logger(), "PositionRMS=[%.3lf,%.3lf,%.3lf]", msg->position_rms[0], msg->position_rms[1], msg->position_rms[2]);
    RCLCPP_INFO(node->get_logger(), "VelocityRMS=[%.3lf,%.3lf,%.3lf]", msg->velocity_rms[0], msg->velocity_rms[1], msg->velocity_rms[2]);
    RCLCPP_INFO(node->get_logger(), "AttitudeRMS=[%.3lf,%.3lf,%.3lf]", msg->attitude_rms[0], msg->attitude_rms[1], msg->attitude_rms[2]);
    RCLCPP_INFO(node->get_logger(), "GpsWeekNumber=%d", msg->gps_week_number);
    RCLCPP_INFO(node->get_logger(), "Alignment=%d\n", msg->alignment);
}

void dumpTimeSyncMessage(const ds_node::msg::TimeSync::SharedPtr &tsmsg)
{
    if (flog)
        fprintf(flog, "%f\n", tsmsg->system_computer_time);

    RCLCPP_INFO(node->get_logger(), "System Computer Time=%f", tsmsg->system_computer_time);
    RCLCPP_INFO(node->get_logger(), "Bias with respect To GPS Time=%f\n", tsmsg->bias_to_gps_time);
}

void dumpGeoidMessage(const ds_node::msg::Geoid::SharedPtr &gmsg)
{
    if (flog)
        fprintf(flog, "%f\n", gmsg->gps_time);

    RCLCPP_INFO(node->get_logger(), "GPSTime=%f", gmsg->gps_time);
    RCLCPP_INFO(node->get_logger(), "GeoidHeight=%f\n", gmsg->geoid_height);
}

void dumpCorrectedIMUMessage(const ds_node::msg::CorrectedIMU::SharedPtr &cimsg)
{

    if (flog)
        fprintf(flog, "%lf\n", cimsg->gps_time_week);

    RCLCPP_INFO(node->get_logger(), "GpsTimeWeek=%f", cimsg->gps_time_week);
    RCLCPP_INFO(node->get_logger(), "Acceleration=[%f,%f,%f]", cimsg->acceleration[0], cimsg->acceleration[1], cimsg->acceleration[2]);
    RCLCPP_INFO(node->get_logger(), "RotationRate=[%f,%f,%f]", cimsg->rotation_rate[0], cimsg->rotation_rate[1], cimsg->rotation_rate[2]);
    RCLCPP_INFO(node->get_logger(), "GpsWeekNumber=%d\n", cimsg->gps_week_number);
}

// %Tag(CALLBACK)%
void dsKalmanCallback(const ds_node::msg::Kalman::SharedPtr kalmsg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a Kalman Filter Navigation message:");

    dumpKalmanMessage(kalmsg);
}

void dsRawIMUCallback(const ds_node::msg::RawIMU::SharedPtr imsg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a Scaled Raw IMU Data message:");

    dumpRawIMUMessage(imsg);
}

void dsSolutionStatusCallback(const ds_node::msg::SolutionStatus::SharedPtr smsg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a Solution Status message:");

    dumpSolutionStatus(smsg);
}

void dsICDCallback(const ds_node::msg::CompactNav::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received an CompactNav message:");

    RCLCPP_INFO(node->get_logger(), "message[%p], week=%d\n", (void *)msg.get(), msg->gps_week_number);
    dumpICDMessage(msg);
}

void PoseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a geometry_msgs::PoseStamped message:");

    RCLCPP_INFO(node->get_logger(), "message[%p], time=%09u:%09u\n", (void *)msg.get(), msg->header.stamp.sec, msg->header.stamp.nanosec);
}

void TwistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a geometry_msgs::TwistStamped message:");

    RCLCPP_INFO(node->get_logger(), "message[%p], time=%09u:%09u\n", (void *)msg.get(), msg->header.stamp.sec, msg->header.stamp.nanosec);
}

void AccelStampedCallback(const geometry_msgs::msg::AccelStamped::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a geometry_msgs::AccelStamped message:");

    RCLCPP_INFO(node->get_logger(), "message[%p], time=%09u:%09u\n", (void *)msg.get(), msg->header.stamp.sec, msg->header.stamp.nanosec);
}
#ifdef ENABLED_GEO_POSE_STAMPED
void GeoPoseStampedCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a geographic_msgs::GeoPoseStamped message:");

    RCLCPP_INFO(node->get_logger(), "message[%p], time=%09u:%09u\n", (void *)msg.get(), msg->header.stamp.sec, msg->header.stamp.nanosec);
}
#endif

void NavSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a sensor_msgs::NavSatFix message:");

    RCLCPP_INFO(node->get_logger(), "message[%p], time=%09u:%09u\n", (void *)msg.get(), msg->header.stamp.sec, msg->header.stamp.nanosec);
}

void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a sensor_msgs::Imu message:");

    RCLCPP_INFO(node->get_logger(), "message[%p], time=%09u:%09u\n", (void *)msg.get(), msg->header.stamp.sec, msg->header.stamp.nanosec);
}

void EulerAttitudeCallback(const ds_node::msg::EulerAttitude::SharedPtr qtemsg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received an EulerAttitude message:");
    RCLCPP_INFO(node->get_logger(), "GpsTimeWeek=%f", qtemsg->gps_time_week);
    RCLCPP_INFO(node->get_logger(), "roll=%f", qtemsg->roll * RAD_TO_DEG);
    RCLCPP_INFO(node->get_logger(), "pitch=%f", qtemsg->pitch * RAD_TO_DEG);
    RCLCPP_INFO(node->get_logger(), "heading=%f\n", qtemsg->heading * RAD_TO_DEG);
}

void dsTimeSyncCallback(const ds_node::msg::TimeSync::SharedPtr tsmsg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a Time Sync message:");

    dumpTimeSyncMessage(tsmsg);
}

void dsGeoidCallback(const ds_node::msg::Geoid::SharedPtr gmsg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a Geoid Height message:");

    dumpGeoidMessage(gmsg);
}

void dsCorrectedIMUCallback(const ds_node::msg::CorrectedIMU::SharedPtr cimsg)
{
    RCLCPP_INFO(node->get_logger(), ">>> Received a Corrected IMU Data message:");

    dumpCorrectedIMUMessage(cimsg);
}

int main(int argc, char **argv)
{
    //    fd_log = open(log_file, O_WRONLY | O_CREAT);
    flog = fopen("~/gpstime.log", "wt");

    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("ds_node_listener");

    auto Kalman_sub = node->create_subscription<ds_node::msg::Kalman>("ds_Kalman", 50, dsKalmanCallback);
    auto RawIMU_sub = node->create_subscription<ds_node::msg::RawIMU>("ds_rawIMU", 50, dsRawIMUCallback);
    auto SolutionStatus_sub = node->create_subscription<ds_node::msg::SolutionStatus>("ds_solutionStatus", 50, dsSolutionStatusCallback);
    auto icd_sub = node->create_subscription<ds_node::msg::CompactNav>("ds_compactNav", 50, dsICDCallback);
    auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 50, PoseStampedCallback);
    auto twist_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 50, TwistStampedCallback);
    auto accel_sub = node->create_subscription<geometry_msgs::msg::AccelStamped>("current_acceleration", 50, AccelStampedCallback);

#ifdef ENABLED_GEO_POSE_STAMPED
    auto geopose_sub = node->create_subscription<geographic_msgs::msg::GeoPoseStamped>("current_geopose", 50, GeoPoseStampedCallback);
#endif

    auto navsatfix_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>("current_navsatfix", 50, NavSatFixCallback);
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("current_imu", 50, ImuCallback);
    auto EulerAttitude_sub = node->create_subscription<ds_node::msg::EulerAttitude>("ds_EulerAttitude", 50, EulerAttitudeCallback);
    auto timeSync_sub = node->create_subscription<ds_node::msg::TimeSync>("ds_timeSync", 50, dsTimeSyncCallback);
    auto geoid_sub = node->create_subscription<ds_node::msg::Geoid>("ds_Geoid", 50, dsGeoidCallback);
    auto CorrectedIMU_sub = node->create_subscription<ds_node::msg::CorrectedIMU>("ds_correctedIMU", 50, dsCorrectedIMUCallback);

    // %EndTag(SUBSCRIBER)%

    rclcpp::spin(node);

    if (flog)
        fclose(flog);

    rclcpp::shutdown();

    return 0;
}
// %EndTag(FULLTEXT)%
