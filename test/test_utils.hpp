/**
 * @file test_utils.hpp
 * @brief Utility functions and helpers for testing
 */

#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

#include <cmath>
#include <random>
#include <chrono>
#include <string>

#include "ds_node/msg/compact_nav.hpp"
#include "ds_node/msg/kalman.hpp"
#include "ds_node/msg/gnss_hmr.hpp"
#include "ds_node/msg/raw_imu.hpp"
#include "ds_node/msg/nmea_gga.hpp"

namespace ds_test
{

/**
 * @brief Generate random floating point number in range [min, max]
 */
template<typename T>
T randomFloat(T min, T max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<T> dis(min, max);
    return dis(gen);
}

/**
 * @brief Generate random integer in range [min, max]
 */
int randomInt(int min, int max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(min, max);
    return dis(gen);
}

/**
 * @brief Create a test CompactNav message with realistic values
 */
ds_node::msg::CompactNav createTestCompactNav()
{
    ds_node::msg::CompactNav msg;
    
    // GPS time (week 2200, ~100 hours into week)
    msg.gps_week = 2200;
    msg.gps_tow = 360000.0;
    
    // Position (New York City area)
    msg.latitude = 0.710584;   // ~40.7 degrees
    msg.longitude = -1.28768;  // ~-73.8 degrees
    msg.altitude = randomFloat(10.0, 200.0);
    
    // Velocity (m/s)
    msg.vel_north = randomFloat(-5.0, 5.0);
    msg.vel_east = randomFloat(-5.0, 5.0);
    msg.vel_down = randomFloat(-2.0, 2.0);
    
    // Attitude (radians)
    msg.roll = randomFloat(-0.3, 0.3);    // ±17 degrees
    msg.pitch = randomFloat(-0.3, 0.3);   // ±17 degrees
    msg.heading = randomFloat(-M_PI, M_PI);
    
    // RMS values (accuracy estimates)
    msg.rms_north = randomFloat(0.1, 2.0);
    msg.rms_east = randomFloat(0.1, 2.0);
    msg.rms_down = randomFloat(0.2, 4.0);
    msg.rms_vn = randomFloat(0.01, 0.5);
    msg.rms_ve = randomFloat(0.01, 0.5);
    msg.rms_vd = randomFloat(0.02, 1.0);
    
    // Status and satellites
    msg.solution_status = randomInt(0, 6);
    msg.num_sats = randomInt(4, 16);
    
    return msg;
}

/**
 * @brief Create a test Kalman message
 */
ds_node::msg::Kalman createTestKalman()
{
    ds_node::msg::Kalman msg;
    
    // GPS time
    msg.gps_week = 2200;
    msg.gps_tow = 360000.0;
    
    // Position
    msg.latitude = 0.710584;
    msg.longitude = -1.28768;
    msg.altitude = randomFloat(10.0, 200.0);
    
    // Velocity
    msg.vel_north = randomFloat(-5.0, 5.0);
    msg.vel_east = randomFloat(-5.0, 5.0);
    msg.vel_down = randomFloat(-2.0, 2.0);
    
    // Attitude
    msg.roll = randomFloat(-0.3, 0.3);
    msg.pitch = randomFloat(-0.3, 0.3);
    msg.heading = randomFloat(-M_PI, M_PI);
    
    // Angular rates
    msg.roll_rate = randomFloat(-0.1, 0.1);
    msg.pitch_rate = randomFloat(-0.1, 0.1);
    msg.yaw_rate = randomFloat(-0.1, 0.1);
    
    // Accelerations
    msg.accel_north = randomFloat(-1.0, 1.0);
    msg.accel_east = randomFloat(-1.0, 1.0);
    msg.accel_down = randomFloat(-1.0, 1.0) + 9.81; // Include gravity
    
    return msg;
}

/**
 * @brief Create a test GNSS HMR message
 */
ds_node::msg::GnssHmr createTestGnssHmr()
{
    ds_node::msg::GnssHmr msg;
    
    // GPS time
    msg.gps_week = 2200;
    msg.gps_tow = 360000.0;
    
    // Position
    msg.latitude = 0.710584;
    msg.longitude = -1.28768;
    msg.altitude = randomFloat(10.0, 200.0);
    
    // Additional GNSS-specific fields
    msg.hdop = randomFloat(0.5, 3.0);
    msg.vdop = randomFloat(0.8, 4.0);
    msg.pdop = randomFloat(1.0, 5.0);
    
    return msg;
}

/**
 * @brief Create a test Raw IMU message
 */
ds_node::msg::RawIMU createTestRawIMU()
{
    ds_node::msg::RawIMU msg;
    
    // GPS time
    msg.gps_week = 2200;
    msg.gps_tow = 360000.0;
    
    // Gyroscope readings (rad/s)
    msg.gyro_x = randomFloat(-0.5, 0.5);
    msg.gyro_y = randomFloat(-0.5, 0.5);
    msg.gyro_z = randomFloat(-0.5, 0.5);
    
    // Accelerometer readings (m/s²)
    msg.accel_x = randomFloat(-2.0, 2.0);
    msg.accel_y = randomFloat(-2.0, 2.0);
    msg.accel_z = randomFloat(8.0, 12.0); // Around gravity
    
    // Temperature (Celsius)
    msg.temperature = randomFloat(15.0, 35.0);
    
    return msg;
}

/**
 * @brief Create a test NMEA GGA message
 */
ds_node::msg::NmeaGGA createTestNmeaGga()
{
    ds_node::msg::NmeaGGA msg;
    
    // UTC time
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    msg.utc_time.hour = tm.tm_hour;
    msg.utc_time.minute = tm.tm_min;
    msg.utc_time.second = tm.tm_sec;
    
    // Position (slightly randomized NYC coordinates)
    msg.latitude = 40.7128 + randomFloat(-0.01, 0.01);
    msg.longitude = -74.0060 + randomFloat(-0.01, 0.01);
    
    // Fix quality
    msg.fix_quality = randomInt(0, 5);
    
    // Number of satellites
    msg.num_satellites = randomInt(4, 16);
    
    // HDOP
    msg.hdop = randomFloat(0.5, 5.0);
    
    // Altitude
    msg.altitude = randomFloat(10.0, 200.0);
    
    // Geoid separation
    msg.geoid_separation = randomFloat(-50.0, 50.0);
    
    return msg;
}

/**
 * @brief Calculate distance between two points on Earth using Haversine formula
 * @param lat1 Latitude of first point in radians
 * @param lon1 Longitude of first point in radians
 * @param lat2 Latitude of second point in radians
 * @param lon2 Longitude of second point in radians
 * @return Distance in meters
 */
double calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0; // Earth's radius in meters
    
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    
    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon/2) * std::sin(dlon/2);
    
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    
    return R * c;
}

/**
 * @brief Convert degrees to radians
 */
double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

/**
 * @brief Convert radians to degrees
 */
double radToDeg(double radians)
{
    return radians * 180.0 / M_PI;
}

/**
 * @brief Generate a valid NMEA GGA sentence
 */
std::string generateNmeaGgaSentence(
    int hour, int minute, int second,
    double latitude, double longitude,
    int fix_quality, int num_sats,
    double hdop, double altitude,
    double geoid_separation)
{
    char sentence[256];
    
    // Convert latitude to NMEA format (DDMM.MMMM)
    double lat_abs = std::abs(latitude);
    int lat_deg = static_cast<int>(lat_abs);
    double lat_min = (lat_abs - lat_deg) * 60.0;
    char lat_hemi = (latitude >= 0) ? 'N' : 'S';
    
    // Convert longitude to NMEA format (DDDMM.MMMM)
    double lon_abs = std::abs(longitude);
    int lon_deg = static_cast<int>(lon_abs);
    double lon_min = (lon_abs - lon_deg) * 60.0;
    char lon_hemi = (longitude >= 0) ? 'E' : 'W';
    
    // Format the sentence
    snprintf(sentence, sizeof(sentence),
             "$GPGGA,%02d%02d%02d,%02d%05.2f,%c,%03d%05.2f,%c,%d,%02d,%.1f,%.1f,M,%.1f,M,,",
             hour, minute, second,
             lat_deg, lat_min, lat_hemi,
             lon_deg, lon_min, lon_hemi,
             fix_quality, num_sats, hdop, altitude, geoid_separation);
    
    // Calculate checksum
    unsigned char checksum = 0;
    for (int i = 1; sentence[i] != '\0'; i++)
    {
        checksum ^= sentence[i];
    }
    
    // Append checksum
    char final_sentence[256];
    snprintf(final_sentence, sizeof(final_sentence), "%s*%02X", sentence, checksum);
    
    return std::string(final_sentence);
}

/**
 * @brief Verify that a quaternion is normalized
 */
bool isQuaternionNormalized(double x, double y, double z, double w, double tolerance = 1e-6)
{
    double norm = std::sqrt(x*x + y*y + z*z + w*w);
    return std::abs(norm - 1.0) < tolerance;
}

/**
 * @brief Verify that a 3x3 covariance matrix is positive semi-definite
 */
bool isCovarianceMatrixValid(const double cov[9], double tolerance = 1e-6)
{
    // Check diagonal elements are non-negative
    if (cov[0] < -tolerance || cov[4] < -tolerance || cov[8] < -tolerance)
        return false;
    
    // Check matrix is symmetric
    if (std::abs(cov[1] - cov[3]) > tolerance ||
        std::abs(cov[2] - cov[6]) > tolerance ||
        std::abs(cov[5] - cov[7]) > tolerance)
        return false;
    
    return true;
}

} // namespace ds_test

#endif // TEST_UTILS_HPP