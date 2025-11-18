/**
 * @file test_ds_convert.cpp
 * @brief Unit tests for ds_convert functions
 */

#include <gtest/gtest.h>
#include <cmath>
#include "ds_convert.h"
#include "ds_node/msg/compact_nav.hpp"
#include "ds_node/msg/euler_attitude.hpp"

class DsConvertTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Initialize test data
        test_nav_msg.gps_week = 2200;
        test_nav_msg.gps_tow = 360000.0; // 100 hours in seconds
        test_nav_msg.latitude = 0.710584;  // ~40.7 degrees in radians
        test_nav_msg.longitude = -1.28768; // ~73.8 degrees in radians
        test_nav_msg.altitude = 100.0;
        test_nav_msg.vel_north = 1.0;
        test_nav_msg.vel_east = 2.0;
        test_nav_msg.vel_down = -0.5;
        test_nav_msg.roll = 0.1;
        test_nav_msg.pitch = 0.2;
        test_nav_msg.heading = 0.3;
        test_nav_msg.rms_north = 0.5;
        test_nav_msg.rms_east = 0.6;
        test_nav_msg.rms_down = 0.7;
        test_nav_msg.rms_vn = 0.1;
        test_nav_msg.rms_ve = 0.15;
        test_nav_msg.rms_vd = 0.08;
        test_nav_msg.solution_status = 1;
        test_nav_msg.num_sats = 12;
    }

    ds_node::msg::CompactNav test_nav_msg;
};

// Test GPS time to Epoch conversion
TEST_F(DsConvertTest, GpsToEpochConversion)
{
    builtin_interfaces::msg::Time epoch_time;
    
    // Test with known GPS week and TOW
    ds::GpsToEpoch(2200, 360000.0, epoch_time);
    
    // Expected Unix timestamp: GPS epoch (315964800) + week * 604800 + TOW
    // 315964800 + 2200 * 604800 + 360000 = 315964800 + 1330560000 + 360000 = 1646884800
    EXPECT_EQ(epoch_time.sec, 1646884800);
    EXPECT_EQ(epoch_time.nanosec, 0);
}

// Test Epoch to GPS time conversion
TEST_F(DsConvertTest, EpochToGpsConversion)
{
    builtin_interfaces::msg::Time epoch_time;
    epoch_time.sec = 1646884800;
    epoch_time.nanosec = 0;
    
    int gps_week;
    double gps_tow;
    
    ds::EpochToGps(epoch_time, gps_week, gps_tow);
    
    EXPECT_EQ(gps_week, 2200);
    EXPECT_DOUBLE_EQ(gps_tow, 360000.0);
}

// Test quaternion product
TEST_F(DsConvertTest, QuaternionProduct)
{
    geometry_msgs::msg::Quaternion q1, q2, q3;
    
    // Identity quaternion
    q1.x = 0.0; q1.y = 0.0; q1.z = 0.0; q1.w = 1.0;
    q2.x = 0.0; q2.y = 0.0; q2.z = 0.0; q2.w = 1.0;
    
    ds::QuatProd(q1, q2, q3);
    
    EXPECT_DOUBLE_EQ(q3.x, 0.0);
    EXPECT_DOUBLE_EQ(q3.y, 0.0);
    EXPECT_DOUBLE_EQ(q3.z, 0.0);
    EXPECT_DOUBLE_EQ(q3.w, 1.0);
}

// Test NED to ENU quaternion conversion
TEST_F(DsConvertTest, NEDToENUQuaternion)
{
    geometry_msgs::msg::Quaternion q;
    
    ds::QuatNED2ENU(q);
    
    // The NED to ENU transformation should produce a specific quaternion
    // This is a 90-degree rotation around Z axis followed by 180-degree around X
    EXPECT_NEAR(q.x, 0.7071068, 1e-6);  // ~1/√2
    EXPECT_NEAR(q.y, 0.7071068, 1e-6);  // ~1/√2
    EXPECT_DOUBLE_EQ(q.z, 0.0);
    EXPECT_DOUBLE_EQ(q.w, 0.0);
}

// Test CompactNav to NavSatFix conversion
TEST_F(DsConvertTest, CompactNavToNavSatFix)
{
    sensor_msgs::msg::NavSatFix nav_sat_fix;
    
    ds::icd_to_NavSatFix(test_nav_msg, nav_sat_fix);
    
    EXPECT_DOUBLE_EQ(nav_sat_fix.latitude, test_nav_msg.latitude * 180.0 / M_PI);
    EXPECT_DOUBLE_EQ(nav_sat_fix.longitude, test_nav_msg.longitude * 180.0 / M_PI);
    EXPECT_DOUBLE_EQ(nav_sat_fix.altitude, test_nav_msg.altitude);
    EXPECT_EQ(nav_sat_fix.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    EXPECT_EQ(nav_sat_fix.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
}

// Test CompactNav to IMU conversion
TEST_F(DsConvertTest, CompactNavToImu)
{
    sensor_msgs::msg::Imu imu_msg;
    
    ds::icd_to_Imu(test_nav_msg, imu_msg);
    
    // Check that orientation quaternion is set (non-zero)
    EXPECT_NE(imu_msg.orientation.x, 0.0);
    EXPECT_NE(imu_msg.orientation.y, 0.0);
    EXPECT_NE(imu_msg.orientation.z, 0.0);
    EXPECT_NE(imu_msg.orientation.w, 0.0);
    
    // Check angular velocity (converted from roll, pitch, heading rates)
    // Note: These might be zero if rates are not provided in CompactNav
    
    // Check linear acceleration (should be zero if not provided)
    EXPECT_DOUBLE_EQ(imu_msg.linear_acceleration.x, 0.0);
    EXPECT_DOUBLE_EQ(imu_msg.linear_acceleration.y, 0.0);
    EXPECT_DOUBLE_EQ(imu_msg.linear_acceleration.z, 0.0);
}

// Test CompactNav to TwistStamped conversion
TEST_F(DsConvertTest, CompactNavToTwistStamped)
{
    geometry_msgs::msg::TwistStamped twist_stamped;
    
    ds::icd_to_TwistStamped(test_nav_msg, twist_stamped);
    
    EXPECT_DOUBLE_EQ(twist_stamped.twist.linear.x, test_nav_msg.vel_east);
    EXPECT_DOUBLE_EQ(twist_stamped.twist.linear.y, test_nav_msg.vel_north);
    EXPECT_DOUBLE_EQ(twist_stamped.twist.linear.z, -test_nav_msg.vel_down); // ENU frame
}

// Test CompactNav to AccelStamped conversion
TEST_F(DsConvertTest, CompactNavToAccelStamped)
{
    geometry_msgs::msg::AccelStamped accel_stamped;
    
    ds::icd_to_AccelStamped(test_nav_msg, accel_stamped);
    
    // Acceleration values should be set based on velocity RMS values
    EXPECT_GT(accel_stamped.accel.linear.x, 0.0);
    EXPECT_GT(accel_stamped.accel.linear.y, 0.0);
    EXPECT_GT(accel_stamped.accel.linear.z, 0.0);
}

// Test geodetic to ECEF conversion
TEST_F(DsConvertTest, GeodeticToECEF)
{
    double lat = 0.710584;  // ~40.7 degrees
    double lon = -1.28768;  // ~73.8 degrees
    double alt = 100.0;
    double r[3];
    
    ds::GeodeticToECEF(lat, lon, alt, r);
    
    // Expected ECEF coordinates for this location
    // Values should be reasonable for the given lat/lon/alt
    EXPECT_GT(r[0], 1000000.0);  // X should be positive in northern hemisphere
    EXPECT_LT(r[1], -4000000.0);  // Y should be negative for western hemisphere
    EXPECT_GT(r[2], 4000000.0);   // Z should be positive for northern hemisphere
}

// Test ECEF to geodetic conversion
TEST_F(DsConvertTest, ECEFToGeodetic)
{
    double r[3] = {1330000.0, -4600000.0, 4200000.0}; // Approximate ECEF for NYC area
    double lat, lon, alt;
    
    ds::ECEFToGeodetic(r, lat, lon, alt);
    
    // Convert back to degrees for easier verification
    double lat_deg = lat * 180.0 / M_PI;
    double lon_deg = lon * 180.0 / M_PI;
    
    EXPECT_NEAR(lat_deg, 40.7, 1.0);   // Should be around 40.7 degrees
    EXPECT_NEAR(lon_deg, -73.8, 1.0);  // Should be around -73.8 degrees
    EXPECT_GT(alt, 0.0);                 // Altitude should be positive
}

// Test DCM ECEF to NED calculation
TEST_F(DsConvertTest, DCM_ECEFToNED)
{
    double lat = 0.710584;  // ~40.7 degrees
    double lon = -1.28768;  // ~73.8 degrees
    double Cen[3][3];
    
    ds::DCM_ECEFToNED(lat, lon, Cen);
    
    // Check that this is a valid DCM (orthogonal matrix)
    // For a valid DCM, rows should be unit vectors
    for (int i = 0; i < 3; i++) {
        double row_norm = sqrt(Cen[i][0]*Cen[i][0] + Cen[i][1]*Cen[i][1] + Cen[i][2]*Cen[i][2]);
        EXPECT_NEAR(row_norm, 1.0, 1e-10);
    }
}

// Test covariance assignment functions
TEST_F(DsConvertTest, CovarianceAssignment)
{
    double rms[3] = {0.1, 0.2, 0.3};
    double cov[9];
    
    ds::AssignDiagCov3(rms, cov);
    
    EXPECT_DOUBLE_EQ(cov[0], 0.01);   // 0.1^2
    EXPECT_DOUBLE_EQ(cov[4], 0.04);   // 0.2^2
    EXPECT_DOUBLE_EQ(cov[8], 0.09);   // 0.3^2
    EXPECT_DOUBLE_EQ(cov[1], 0.0);    // Off-diagonal
    EXPECT_DOUBLE_EQ(cov[2], 0.0);    // Off-diagonal
    EXPECT_DOUBLE_EQ(cov[3], 0.0);    // Off-diagonal
    EXPECT_DOUBLE_EQ(cov[5], 0.0);    // Off-diagonal
    EXPECT_DOUBLE_EQ(cov[6], 0.0);    // Off-diagonal
    EXPECT_DOUBLE_EQ(cov[7], 0.0);    // Off-diagonal
}

// Test 6x6 covariance assignment
TEST_F(DsConvertTest, Covariance6Assignment)
{
    double rms1[3] = {0.1, 0.2, 0.3};
    double rms2[3] = {0.4, 0.5, 0.6};
    double cov[36];
    
    ds::AssignDiagCov6(rms1, rms2, cov);
    
    EXPECT_DOUBLE_EQ(cov[0], 0.01);   // 0.1^2
    EXPECT_DOUBLE_EQ(cov[7], 0.04);   // 0.2^2
    EXPECT_DOUBLE_EQ(cov[14], 0.09);  // 0.3^2
    EXPECT_DOUBLE_EQ(cov[21], 0.16);  // 0.4^2
    EXPECT_DOUBLE_EQ(cov[28], 0.25);  // 0.5^2
    EXPECT_DOUBLE_EQ(cov[35], 0.36);  // 0.6^2
    
    // Check that off-diagonal elements are zero
    for (int i = 0; i < 36; i++) {
        if (i != 0 && i != 7 && i != 14 && i != 21 && i != 28 && i != 35) {
            EXPECT_DOUBLE_EQ(cov[i], 0.0);
        }
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}