#ifndef _DS_NMEA_H
#define _DS_NMEA_H

#include "ds_node/msg/nmea_gga.hpp"

//-----------------------------------------------------------------------------
// Checks NMEA message checksum validity
// @return true if valid

bool nmeaChecksum(const char *msg);

//-----------------------------------------------------------------------------
// Parses NMEA GGA string into the decoded message format
void parseNmeaGga(const char *msg, ds_node::msg::NmeaGGA &gga);

#endif