//
// Copyright 2010-2011,2014 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef INCLUDED_GPS_CTRL_HPP
#define INCLUDED_GPS_CTRL_HPP

#include <uhd/types/serial.hpp>
#include <uhd/types/sensors.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/function.hpp>
#include <vector>

namespace uhd{

class UHD_API gps_ctrl : boost::noncopyable{
public:
  typedef boost::shared_ptr<gps_ctrl> sptr;

  virtual ~gps_ctrl(void) = 0;

  /*!
   * Make a GPS config for internal GPSDOs or generic NMEA GPS devices
   */
  static sptr make(uart_iface::sptr uart);

  /*!
   * Retrieve the list of sensors this GPS object provides
   */
  virtual std::vector<std::string> get_sensors(void) = 0;

  /*!
   * Retrieve the named sensor
   */
  virtual uhd::sensor_value_t get_sensor(std::string key) = 0;

  /*!
   * Tell you if there's a supported GPS connected or not
   * \return true if a supported GPS is connected
   */
  virtual bool gps_detected(void) = 0;

  /*!
   * Tell if the GPSDO is a LEA-M8F
   * \return true if the LEA-M8F has been detected
   */
  virtual bool gps_detected_lea_m8f(void) = 0;

  //TODO: other fun things you can do with a GPS.

};

/*! The UBX-NAV-SOL message structure from the
 *  u-blox UBX protocol
 */
struct ubx_nav_sol_t
{
    uint32_t  itow;           // ms GPS Millisecond Time of Week
    int32_t   frac;           // ns remainder of rounded ms above
    int16_t   week;           // GPS week
    uint8_t   GPSfix;         // GPSfix Type, range 0..6
    uint8_t   Flags;          // Navigation Status Flags
    int32_t   ECEF_X;         // cm ECEF X coordinate
    int32_t   ECEF_Y;         // cm ECEF Y coordinate
    int32_t   ECEF_Z;         // cm ECEF Z coordinate
    int32_t   PAcc;           // cm 3D Position Accuracy Estimate
    int32_t   ECEFVX;         // cm/s ECEF X velocity
    int32_t   ECEFVY;         // cm/s ECEF Y velocity
    int32_t   ECEFVZ;         // cm/s ECEF Z velocity
    uint32_t  SAcc;           // cm/s Speed Accuracy Estimate
    uint16_t  PDOP;           // 0.01 Position DOP
    uint8_t   res1;           // reserved
    uint8_t   numSV;          // Number of SVs used in navigation solution
    uint32_t  res2;           // reserved
} __attribute__((packed));

} //namespace uhd

#endif /* INCLUDED_GPS_CTRL_HPP */
