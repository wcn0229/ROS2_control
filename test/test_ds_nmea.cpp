/**
 * @file test_ds_nmea.cpp
 * @brief Unit tests for ds_nmea functions
 */

#include <gtest/gtest.h>
#include <cstring>
#include "ds_nmea.h"
#include "ds_node/msg/nmea_gga.hpp"

class DsNmeaTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Valid GGA message with correct checksum
        valid_gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        
        // Valid GGA message with different coordinates
        valid_gga2 = "$GPGGA,134658,4807.123,N,01131.456,E,1,12,1.0,550.0,M,47.0,M,,*6B";
        
        // Invalid GGA message (wrong checksum)
        invalid_gga_checksum = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48";
        
        // Invalid GGA message (missing fields)
        invalid_gga_missing = "$GPGGA,123519,4807.038,N,,*7A";
        
        // GGA message with zero coordinates
        zero_coords_gga = "$GPGGA,123519,0000.000,N,00000.000,E,1,08,0.9,0.0,M,0.0,M,,*1A";
    }

    std::string valid_gga;
    std::string valid_gga2;
    std::string invalid_gga_checksum;
    std::string invalid_gga_missing;
    std::string zero_coords_gga;
};

// Test NMEA checksum validation with valid message
TEST_F(DsNmeaTest, ValidChecksum)
{
    bool result = nmeaChecksum(valid_gga.c_str());
    EXPECT_TRUE(result);
}

// Test NMEA checksum validation with invalid checksum
TEST_F(DsNmeaTest, InvalidChecksum)
{
    bool result = nmeaChecksum(invalid_gga_checksum.c_str());
    EXPECT_FALSE(result);
}

// Test NMEA GGA parsing with valid message
TEST_F(DsNmeaTest, ParseValidGga)
{
    ds_node::msg::NmeaGga gga_msg;
    
    parseNmeaGga(valid_gga.c_str(), gga_msg);
    
    // Check UTC time
    EXPECT_EQ(gga_msg.utc_time.hour, 12);
    EXPECT_EQ(gga_msg.utc_time.minute, 35);
    EXPECT_EQ(gga_msg.utc_time.second, 19);
    
    // Check latitude (4807.038,N -> 48.1173 degrees)
    EXPECT_NEAR(gga_msg.latitude, 48.1173, 0.0001);
    
    // Check longitude (01131.000,E -> 11.5167 degrees)
    EXPECT_NEAR(gga_msg.longitude, 11.5167, 0.0001);
    
    // Check fix quality
    EXPECT_EQ(gga_msg.fix_quality, 1);
    
    // Check number of satellites
    EXPECT_EQ(gga_msg.num_satellites, 8);
    
    // Check HDOP
    EXPECT_NEAR(gga_msg.hdop, 0.9, 0.01);
    
    // Check altitude
    EXPECT_NEAR(gga_msg.altitude, 545.4, 0.1);
    
    // Check geoid separation
    EXPECT_NEAR(gga_msg.geoid_separation, 46.9, 0.1);
}

// Test NMEA GGA parsing with zero coordinates
TEST_F(DsNmeaTest, ParseZeroCoordinatesGga)
{
    ds_node::msg::NmeaGga gga_msg;
    
    parseNmeaGga(zero_coords_gga.c_str(), gga_msg);
    
    // Check coordinates are zero
    EXPECT_NEAR(gga_msg.latitude, 0.0, 0.0001);
    EXPECT_NEAR(gga_msg.longitude, 0.0, 0.0001);
    EXPECT_NEAR(gga_msg.altitude, 0.0, 0.1);
    EXPECT_NEAR(gga_msg.geoid_separation, 0.0, 0.1);
}

// Test NMEA GGA parsing with invalid checksum (should still parse data)
TEST_F(DsNmeaTest, ParseInvalidChecksumGga)
{
    ds_node::msg::NmeaGga gga_msg;
    
    // This should still parse the data even with invalid checksum
    parseNmeaGga(invalid_gga_checksum.c_str(), gga_msg);
    
    // Check that data was parsed (checksum validation is separate)
    EXPECT_EQ(gga_msg.utc_time.hour, 12);
    EXPECT_EQ(gga_msg.utc_time.minute, 35);
    EXPECT_NEAR(gga_msg.latitude, 48.1173, 0.0001);
}

// Test NMEA GGA parsing with missing fields
TEST_F(DsNmeaTest, ParseMissingFieldsGga)
{
    ds_node::msg::NmeaGga gga_msg;
    
    parseNmeaGga(invalid_gga_missing.c_str(), gga_msg);
    
    // Should handle missing fields gracefully
    EXPECT_EQ(gga_msg.utc_time.hour, 12);
    EXPECT_EQ(gga_msg.utc_time.minute, 35);
    // Other fields should have default/invalid values
}

// Test southern hemisphere coordinates
TEST_F(DsNmeaTest, SouthernHemisphereGga)
{
    std::string south_gga = "$GPGGA,123519,3347.123,S,15113.456,E,1,08,0.9,100.0,M,50.0,M,,*3A";
    ds_node::msg::NmeaGga gga_msg;
    
    parseNmeaGga(south_gga.c_str(), gga_msg);
    
    // Latitude should be negative for southern hemisphere
    EXPECT_LT(gga_msg.latitude, 0.0);
    // Convert from DM to decimal degrees: 3347.123,S -> -33.7854
    EXPECT_NEAR(gga_msg.latitude, -33.7854, 0.0001);
}

// Test western hemisphere coordinates
TEST_F(DsNmeaTest, WesternHemisphereGga)
{
    std::string west_gga = "$GPGGA,123519,4807.123,N,07413.456,W,1,08,0.9,100.0,M,50.0,M,,*2C";
    ds_node::msg::NmeaGga gga_msg;
    
    parseNmeaGga(west_gga.c_str(), gga_msg);
    
    // Longitude should be negative for western hemisphere
    EXPECT_LT(gga_msg.longitude, 0.0);
    // Convert from DM to decimal degrees: 07413.456,W -> -74.2243
    EXPECT_NEAR(gga_msg.longitude, -74.2243, 0.0001);
}

// Test different fix qualities
TEST_F(DsNmeaTest, DifferentFixQualitiesGga)
{
    // Fix quality 0: Invalid
    std::string invalid_fix = "$GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,*46";
    ds_node::msg::NmeaGga gga_msg;
    
    parseNmeaGga(invalid_fix.c_str(), gga_msg);
    EXPECT_EQ(gga_msg.fix_quality, 0);
    
    // Fix quality 2: DGPS
    std::string dgps_fix = "$GPGGA,123519,4807.038,N,01131.000,E,2,08,0.9,545.4,M,46.9,M,,*44";
    parseNmeaGga(dgps_fix.c_str(), gga_msg);
    EXPECT_EQ(gga_msg.fix_quality, 2);
    
    // Fix quality 4: RTK
    std::string rtk_fix = "$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*42";
    parseNmeaGga(rtk_fix.c_str(), gga_msg);
    EXPECT_EQ(gga_msg.fix_quality, 4);
}

// Test edge case with minimal valid GGA message
TEST_F(DsNmeaTest, MinimalValidGga)
{
    std::string minimal_gga = "$GPGGA,123519,,,,0,00,99.9,,M,,M,,*4F";
    ds_node::msg::NmeaGga gga_msg;
    
    parseNmeaGga(minimal_gga.c_str(), gga_msg);
    
    EXPECT_EQ(gga_msg.utc_time.hour, 12);
    EXPECT_EQ(gga_msg.utc_time.minute, 35);
    EXPECT_EQ(gga_msg.utc_time.second, 19);
    EXPECT_EQ(gga_msg.fix_quality, 0);
    EXPECT_EQ(gga_msg.num_satellites, 0);
    EXPECT_NEAR(gga_msg.hdop, 99.9, 0.1);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}