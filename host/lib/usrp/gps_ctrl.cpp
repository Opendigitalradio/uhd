//
// Copyright 2010-2011,2014-2015 Ettus Research LLC
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

#include <uhd/usrp/gps_ctrl.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/sensors.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/cstdint.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/tokenizer.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>

#include "boost/tuple/tuple.hpp"
#include "boost/foreach.hpp"

using namespace uhd;
using namespace boost::gregorian;
using namespace boost::posix_time;
using namespace boost::algorithm;
using namespace boost::this_thread;

/*!
 * A NMEA and UBX parser for the GPS serial interface.
 */
class gps_ctrl_parser {
  private:
    std::deque<char> gps_data_input;

    std::string parse_ubx()
    {
      // Assumptions:
      // The deque now contains an UBX message in the format
      //  \xb6\x62<CLASS><ID><LEN><PAYLOAD><CRC>
      // where
      //  <CLASS> is 1 byte
      //  <ID>    is 1 byte
      //  <LEN>   is 2 bytes (little-endian), length of <PAYLOAD>
      //  <CRC>   is 2 bytes

      uint8_t ck_a = 0;
      uint8_t ck_b = 0;


      if (gps_data_input.size() >= 8) {
        uint8_t len_lo = gps_data_input[4];
        uint8_t len_hi = gps_data_input[5];
        size_t len     = len_lo | (len_hi << 8);

        if (gps_data_input.size() >= len + 8) {
          /*
          std::cerr << "DATA: ";
          for (size_t i=0; i < gps_data_input.size(); i++) {
            uint8_t dat = gps_data_input[i];
            std::cerr << boost::format("%02x ") % (unsigned int)dat;
          }
          std::cerr << std::endl;
          */

          uint8_t ck_a_packet = gps_data_input[len+6];
          uint8_t ck_b_packet = gps_data_input[len+7];

          // Range over which CRC is calculated is <CLASS><ID><LEN><PAYLOAD>
          for (size_t i = 2; i < len+6; i++) {
            ck_a += (uint8_t)(gps_data_input[i]);
            ck_b += ck_a;
          }

          std::string msg(gps_data_input.begin(), gps_data_input.begin() + (len + 8));
          gps_data_input.erase(gps_data_input.begin(), gps_data_input.begin() + (len + 8));

          if (ck_a == ck_a_packet and ck_b == ck_b_packet) {
            return msg;
          }
        }
      }

      return std::string();
    }

    std::string parse_nmea()
    {
      // Assumptions:
      // The deque now contains an NMEA message in the format
      //  $G.................*XX<CR><LF>
      // the checksum XX is dropped from the message

      std::deque<char>::iterator star;
      star = std::find(gps_data_input.begin() + 2, gps_data_input.end(), '*');
      if (star != gps_data_input.end()) {
        std::string msg(gps_data_input.begin(), star);

        // The parser will take care of the leftover *XX<CR><LF>
        gps_data_input.erase(gps_data_input.begin(), star);
        return msg;
      }

      return std::string();
    }

  public:
    template <class InputIterator>
      void push_data(InputIterator first, InputIterator last)
      {
        gps_data_input.insert(gps_data_input.end(), first, last);
      }

    std::string get_next_message()
    {
      while (gps_data_input.size() >= 2) {
        char header1 = gps_data_input[0];
        char header2 = gps_data_input[1];

        std::string parsed;

        if (header1 == '$' and header2 == 'G') {
          parsed = parse_nmea();
        }
        else if (header1 == '\xb5' and header2 == '\x62') {
          parsed = parse_ubx();
        }

        if (parsed.empty()) {
          gps_data_input.pop_front();
        }
        else {
          return parsed;
        }
      }
      return std::string();
    }

    size_t size() { return gps_data_input.size(); }
};

gps_ctrl::~gps_ctrl(void){
    /* NOP */
}

/*!
 * A control for GPSDO devices
 */
class gps_ctrl_impl : public gps_ctrl{
private:
  std::map<std::string, boost::tuple<std::string, boost::system_time, bool> > sensors;

  gps_ctrl_parser gps_parser;

  std::string get_cached_sensor(const std::string sensor, const int freshness, const bool once, const bool touch=true) {
    boost::system_time time = boost::get_system_time();
    try {
      // this is nasty ...
      //std::cout << boost::format("Requested %s - seen? ") % sensor << sensors[sensor].get<2>() << " once? " << once << std::endl;
      if(time - sensors[sensor].get<1>() < milliseconds(freshness) && (!once or !sensors[sensor].get<2>())) {
        sensors[sensor] = boost::make_tuple(sensors[sensor].get<0>(), sensors[sensor].get<1>(), touch);
        return sensors[sensor].get<0>();
      } else {
          return update_cached_sensors(sensor);
      }
    } catch(std::exception &e) {
      UHD_MSG(warning) << "get_cached_sensor: " << e.what() << std::endl;
    }
    return std::string();
  }

    static bool is_nmea_checksum_ok(std::string nmea)
    {
        if (nmea.length() < 5 || nmea[0] != '$' || nmea[nmea.length()-3] != '*')
            return false;

        std::stringstream ss;
        boost::uint32_t string_crc;
        boost::uint32_t calculated_crc = 0;

        // get crc from string
        ss << std::hex << nmea.substr(nmea.length()-2, 2);
        ss >> string_crc;

        // calculate crc
        for (size_t i = 1; i < nmea.length()-3; i++)
            calculated_crc ^= nmea[i];

        // return comparison
        return (string_crc == calculated_crc);
    }

  std::string update_cached_sensors(const std::string sensor) {
    if (gps_detected() && gps_type == GPS_TYPE_INTERNAL_GPSDO) {

      const std::list<std::string> list = boost::assign::list_of("GPGGA")("GPRMC")("SERVO");

      static const boost::regex status_regex("\\d\\d-\\d\\d-\\d\\d");
      static const boost::regex gp_msg_regex("^\\$GP.*,\\*[0-9A-F]{2}$");
      std::map<std::string,std::string> msgs;

      // Get all GPSDO messages available
      // Creating a map here because we only want the latest of each message type
      for (std::string msg = _recv(); msg.length(); msg = _recv())
      {
          // Strip any end of line characters
          erase_all(msg, "\r");
          erase_all(msg, "\n");

          if (msg.length() < 6)
          {
              UHD_LOGV(regularly) << __FUNCTION__ << ": Short NMEA string: " << msg << std::endl;
              continue;
          }

          // Look for SERVO message
          if (boost::regex_search(msg, status_regex, boost::regex_constants::match_continuous))
          {
              msgs["SERVO"] = msg;
          }
          else if (boost::regex_match(msg, gp_msg_regex) and is_nmea_checksum_ok(msg))
          {
              msgs[msg.substr(1,5)] = msg;
          }
          else
          {
              UHD_LOGV(regularly) << __FUNCTION__ << ": Malformed NMEA string: " << msg << std::endl;
          }
      }

      boost::system_time time = boost::get_system_time();

      // Update sensors with newly read data
      BOOST_FOREACH(std::string key, list) {
        if (msgs[key].length())
          sensors[key] = boost::make_tuple(msgs[key], time, !sensor.compare(key));
      }

      // Return requested sensor if it was updated
      if (msgs[sensor].length())
        return msgs[sensor];

      return std::string();
    }
    else if (gps_detected() && gps_type == GPS_TYPE_LEA_M8F) {
      const std::list<std::string> list = boost::assign::list_of("GNGGA")("GNRMC")("TIMELOCK");

      // Concatenate all incoming data into the deque
      for (std::string msg = _recv(); msg.length() > 0; msg = _recv())
      {
        gps_parser.push_data(msg.begin(), msg.end());
      }

      std::map<std::string,std::string> msgs;

      // Get all GPSDO messages available
      // Creating a map here because we only want the latest of each message type
      for (std::string msg = gps_parser.get_next_message(); not msg.empty(); msg = gps_parser.get_next_message())
      {
        /*
        if (msg[0] != '$') {
          std::stringstream ss;
          ss << "Got message ";
          for (size_t m = 0; m < msg.size(); m++) {
            ss << std::hex << (unsigned int)(unsigned char)msg[m] << " " << std::dec;
          }
          UHD_MSG(warning) << ss.str() << ":" << std::endl;
        }
        // */

        const uint8_t tim_tos_head[4] = {0xb5, 0x62, 0x0D, 0x12};
        const std::string tim_tos_head_str(reinterpret_cast<const char *>(tim_tos_head), 4);

        // Try to get NMEA first
        if (msg[0] == '$') {
          msgs[msg.substr(1,5)] = msg;
        }
        else if (msg.find(tim_tos_head_str) == 0 and msg.length() == 56 + 8) {
            // header size == 6, field offset == 4, 32-bit field
            uint8_t flags1 = msg[6 + 4];
            uint8_t flags2 = msg[6 + 5];
            uint8_t flags3 = msg[6 + 5];
            uint8_t flags4 = msg[6 + 5];

            uint32_t flags = flags1 | (flags2 << 8) | (flags3 << 16) | (flags4 << 24);
            /* bits in flags are:
               leapNow 0
               leapSoon 1
               leapPositive 2
               timeInLimit 3
               intOscInLimit 4
               extOscInLimit 5
               gnssTimeValid 6
               UTCTimeValid 7
               DiscSrc 10
               raim 11
               cohPulse 12
               lockedPulse 13
               */

            bool lockedPulse   = (flags & (1 << 13));
            bool timeInLimit   = (flags & (1 << 3));
            bool intOscInLimit = (flags & (1 << 4));

            if (lockedPulse and timeInLimit and intOscInLimit) {
                msgs["TIMELOCK"] = "TIME LOCKED";
            }
            else {
                std::stringstream ss;
                ss <<
                (lockedPulse   ? "" : "no" ) << "lockedPulse " <<
                (timeInLimit   ? "" : "no" ) << "timeInLimit " <<
                (intOscInLimit ? "" : "no" ) << "intOscInLimit ";

                msgs["TIMELOCK"] = ss.str();
            }
        }
        else if (msg[0] == '\xb5' and msg[1] == '\x62') { /* Ignore unsupported UBX message */ }
        else {
          std::stringstream ss;
          ss << "Unknown message ";
          for (size_t m = 0; m < msg.size(); m++) {
            ss << std::hex << (unsigned int)(unsigned char)msg[m] << " " << std::dec;
          }
          UHD_MSG(warning) << ss.str() << ":" << std::endl << msg << std::endl;
        }
      }

      boost::system_time time = boost::get_system_time();

      // Update sensors with newly read data
      BOOST_FOREACH(std::string key, list) {
        if (msgs[key].length()) {
          sensors[key] = boost::make_tuple(msgs[key], time, !sensor.compare(key));
        }
      }

      // Return requested sensor if it was updated
      if (msgs[sensor].length())
        return msgs[sensor];

      return std::string();
    }
    else {
      UHD_MSG(error) << "get_stat(): unsupported GPS or no GPS detected" << std::endl;
      return std::string();
    }
  }

public:
  gps_ctrl_impl(uart_iface::sptr uart){
    _uart = uart;


    std::string reply;
    bool i_heard_some_nmea = false, i_heard_something_weird = false;
    gps_type = GPS_TYPE_NONE;

    //first we look for an internal GPSDO
    _flush(); //get whatever junk is in the rx buffer right now, and throw it away
    _send("HAAAY GUYYYYS\n"); //to elicit a response from the GPSDO

    // try to init LEA-M8F
    init_lea_m8f();

    //wait for _send(...) to return
    sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));

    //then we loop until we either timeout, or until we get a response that indicates we're a JL device
    const boost::system_time comm_timeout = boost::get_system_time() + milliseconds(GPS_COMM_TIMEOUT_MS);
    while(boost::get_system_time() < comm_timeout) {
      reply = _recv();
      //UHD_MSG(warning) << "Received " << reply << std::endl;
      if(reply.find("Command Error") != std::string::npos) {
        gps_type = GPS_TYPE_INTERNAL_GPSDO;
        break;
      }
      else if(reply.substr(0, 3) == "$GP") i_heard_some_nmea = true; //but keep looking for that "Command Error" response
      else if(reply.substr(0, 2) == "\xB5""\x62" or
              reply.substr(0, 3) == "$GN" ) {
          // The u-blox LEA-M8F outputs UBX protocol messages
          gps_type = GPS_TYPE_LEA_M8F;
          i_heard_some_nmea = false;
          break;
      }
      else if(reply.length() != 0) i_heard_something_weird = true; //probably wrong baud rate
    }

    if((i_heard_some_nmea) && (gps_type != GPS_TYPE_INTERNAL_GPSDO)) gps_type = GPS_TYPE_GENERIC_NMEA;

    if((gps_type == GPS_TYPE_NONE) && i_heard_something_weird) {
      UHD_MSG(error) << "GPS invalid reply \"" << reply << "\", assuming none available" << std::endl;
    }

    switch(gps_type) {
    case GPS_TYPE_INTERNAL_GPSDO:
      UHD_MSG(status) << "Found an internal GPSDO" << std::endl;
      init_gpsdo();
      break;

    case GPS_TYPE_LEA_M8F:
      UHD_MSG(status) << "Found an internal u-blox LEA-M8F GPSDO" << std::endl;
      init_lea_m8f();
      break;

    case GPS_TYPE_GENERIC_NMEA:
      UHD_MSG(status) << "Found a generic NMEA GPS device" << std::endl;
      break;

    case GPS_TYPE_NONE:
    default:
      UHD_MSG(status) << "No GPSDO found" << std::endl;
      break;

    }
  }

  ~gps_ctrl_impl(void){
    /* NOP */
  }

  //return a list of supported sensors
  std::vector<std::string> get_sensors(void) {
    std::vector<std::string> ret = boost::assign::list_of
        ("gps_gpgga")
        ("gps_gprmc")
        ("gps_gngga")
        ("gps_gnrmc")
        ("gps_time")
        ("gps_locked")
        ("gps_servo")
        ("gps_timelock");
    return ret;
  }

  uhd::sensor_value_t get_sensor(std::string key) {
    if(key == "gps_gpgga"
    or key == "gps_gprmc"
    or key == "gps_gngga"
    or key == "gps_gnrmc"
    or key == "gps_timelock" ) {
        return sensor_value_t(
                 boost::to_upper_copy(key),
                 get_cached_sensor(boost::to_upper_copy(key.substr(4,8)), GPS_NMEA_NORMAL_FRESHNESS, false, false),
                 "");
    }
    else if(key == "gps_time") {
        return sensor_value_t("GPS epoch time", int(get_epoch_time()), "seconds");
    }
    else if(key == "gps_locked") {
        return sensor_value_t("GPS lock status", locked(), "locked", "unlocked");
    }
    else if(key == "gps_servo") {
        return sensor_value_t("GPS servo status", get_servo(), "");
    }
    else {
        throw uhd::value_error("gps ctrl get_sensor unknown key: " + key);
    }
  }

private:
  void init_gpsdo(void) {
    //issue some setup stuff so it spits out the appropriate data
    //none of these should issue replies so we don't bother looking for them
    //we have to sleep between commands because the JL device, despite not acking, takes considerable time to process each command.
     sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));
    _send("SYST:COMM:SER:ECHO OFF\n");
     sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));
    _send("SYST:COMM:SER:PRO OFF\n");
     sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));
    _send("GPS:GPGGA 1\n");
     sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));
    _send("GPS:GGAST 0\n");
     sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));
    _send("GPS:GPRMC 1\n");
     sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));
    _send("SERV:TRAC 0\n");
     sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));
  }

  void init_lea_m8f(void) {
      // Send a GNSS-only hotstart to make sure we're not in holdover right now
      const uint8_t cfg_rst_hotstart[12] = {0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68};
      _send(std::string(reinterpret_cast<const char *>(cfg_rst_hotstart), sizeof(cfg_rst_hotstart)));

      // Give time to module to reboot
      sleep(milliseconds(2000));

      // Enable the UBX-TIM-TOS and the $GNRMC messages
      const uint8_t en_tim_tos[11] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0d, 0x12, 0x01, 0x2a, 0x8b};
      _send(std::string(reinterpret_cast<const char *>(en_tim_tos), sizeof(en_tim_tos)));

      const uint8_t en_gnrmc[11]   = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x04, 0x01, 0xff, 0x18};
      _send(std::string(reinterpret_cast<const char *>(en_gnrmc), sizeof(en_gnrmc)));
  }

  //retrieve a raw NMEA sentence
  std::string get_nmea(std::string msgtype) {
    std::string reply;

    const boost::system_time comm_timeout = boost::get_system_time() + milliseconds(GPS_COMM_TIMEOUT_MS);
    while(boost::get_system_time() < comm_timeout) {
        if(! (msgtype.compare("GPRMC") || msgtype.compare("GNRMC")) ) {
          reply = get_cached_sensor(msgtype, GPS_NMEA_FRESHNESS, true);
        }
        else {
          reply = get_cached_sensor(msgtype, GPS_NMEA_LOW_FRESHNESS, false);
        }
        if(reply.size()) {
          if(reply.substr(1, 5) == msgtype) return reply;
        }
        boost::this_thread::sleep(milliseconds(GPS_TIMEOUT_DELAY_MS));
    }
    throw uhd::value_error(str(boost::format("get_nmea(): no %s message found") % msgtype));
  }

  //helper function to retrieve a field from an NMEA sentence
  std::string get_token(std::string sentence, size_t offset) {
    boost::tokenizer<boost::escaped_list_separator<char> > tok(sentence);
    std::vector<std::string> toked;
    tok.assign(sentence); //this can throw
    toked.assign(tok.begin(), tok.end());

    if(toked.size() <= offset) {
        throw uhd::value_error(str(boost::format("Invalid response \"%s\"") % sentence));
    }
    return toked[offset];
  }

  ptime get_time(void) {
    _flush();
    int error_cnt = 0;
    const int GPS_TIMEVALID_TIMEOUT_MS = 60000;
    ptime gps_time;
    const boost::system_time valid_time_timeout = boost::get_system_time() + milliseconds(GPS_TIMEVALID_TIMEOUT_MS);
    try {
        while(boost::get_system_time() < valid_time_timeout) {
            std::string reply;
            if (gps_type == GPS_TYPE_LEA_M8F) {
                reply = get_nmea("GNRMC");
            }
            else {
                reply = get_nmea("GPRMC");
            }

            std::string datestr = get_token(reply, 9);
            std::string timestr = get_token(reply, 1);

            if (datestr.size() == 0 or timestr.size() == 0) {
                error_cnt++;
                continue;
            }

            //just trust me on this one
            gps_time = ptime( date(
                             greg_year(boost::lexical_cast<int>(datestr.substr(4, 2)) + 2000),
                             greg_month(boost::lexical_cast<int>(datestr.substr(2, 2))),
                             greg_day(boost::lexical_cast<int>(datestr.substr(0, 2)))
                           ),
                          hours(  boost::lexical_cast<int>(timestr.substr(0, 2)))
                        + minutes(boost::lexical_cast<int>(timestr.substr(2, 2)))
                        + seconds(boost::lexical_cast<int>(timestr.substr(4, 2)))
                     );
            UHD_MSG(warning) << "get_time ok: " << error_cnt << std::endl;
            return gps_time;

        }
    } catch(std::exception &e) {
        UHD_MSG(error) << "get_time: " << e.what() << std::endl;
        _flush();
    }
    UHD_MSG(warning) << "get_time err: " << error_cnt << std::endl;
    throw uhd::value_error("Timeout after no valid message found");

    return gps_time; //keep gcc from complaining
  }

  time_t get_epoch_time(void) {
      return (get_time() - from_time_t(0)).total_seconds();
  }

  bool gps_detected_lea_m8f(void) {
    return (gps_type == GPS_TYPE_LEA_M8F);
  }

  bool gps_detected(void) {
    return (gps_type != GPS_TYPE_NONE);
  }

  bool locked(void) {
    int error_cnt = 0;
    while(error_cnt < 3) {
        try {
            std::string reply;
            if (gps_type == GPS_TYPE_LEA_M8F) {
               reply = get_cached_sensor("TIMELOCK", GPS_LOCK_FRESHNESS, false, false);
               return reply == "TIME LOCKED";
            }
            else {
               reply = get_cached_sensor("GPGGA", GPS_LOCK_FRESHNESS, false, false);
               if(reply.size() <= 1) return false;
               return (get_token(reply, 6) != "0");
            }

        } catch(std::exception &e) {
            UHD_MSG(warning) << "locked: " << e.what() << std::endl;
            error_cnt++;
        }
    }
    throw uhd::value_error("Timeout after no valid message found");
    return false;
  }

  std::string get_servo(void) {

    //enable servo reporting
    _send("SERV:TRAC 1\n");
    sleep(milliseconds(GPSDO_COMMAND_DELAY_MS));

    std::string reply;

    const boost::system_time comm_timeout = boost::get_system_time() + milliseconds(GPS_COMM_TIMEOUT_MS);
    while(boost::get_system_time() < comm_timeout) {
        reply = get_cached_sensor("SERVO", GPS_NMEA_LOW_FRESHNESS, false);
        if(reply.size())
        {
            //disable it before leaving function
            _send("SERV:TRAC 0\n");
            return reply;
        }
        boost::this_thread::sleep(milliseconds(GPS_TIMEOUT_DELAY_MS));
    }
    throw uhd::value_error("get_stat(): no servo message found");
  }

  uart_iface::sptr _uart;

  void _flush(void){
    while (not _uart->read_uart(0.0).empty()){
        //NOP
    }
  }

  std::string _recv(double timeout = GPS_TIMEOUT_DELAY_MS/1000.){
      return _uart->read_uart(timeout);
  }

  void _send(const std::string &buf){
      return _uart->write_uart(buf);
  }

  enum {
    GPS_TYPE_INTERNAL_GPSDO,
    GPS_TYPE_LEA_M8F,
    GPS_TYPE_GENERIC_NMEA,
    GPS_TYPE_NONE
  } gps_type;

  static const int GPS_COMM_TIMEOUT_MS = 2300;
  static const int GPS_NMEA_FRESHNESS = 10;
  static const int GPS_NMEA_LOW_FRESHNESS = 2500;
  static const int GPS_NMEA_NORMAL_FRESHNESS = 1000;
  static const int GPS_SERVO_FRESHNESS = 2500;
  static const int GPS_LOCK_FRESHNESS = 200;
  static const int GPS_TIMEOUT_DELAY_MS = 200;
  static const int GPSDO_COMMAND_DELAY_MS = 200;
};

/***********************************************************************
 * Public make function for the GPS control
 **********************************************************************/
gps_ctrl::sptr gps_ctrl::make(uart_iface::sptr uart){
    return sptr(new gps_ctrl_impl(uart));
}
