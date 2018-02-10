//
// Copyright 2010-2011,2014-2016 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/usrp/gps_ctrl.hpp>

#include <uhd/utils/log.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/sensors.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/tokenizer.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time.hpp>
#include <boost/tuple/tuple.hpp>
#include <ctime>
#include <string>
#include <thread>
#include <chrono>
#include <stdint.h>

using namespace uhd;
using namespace boost::posix_time;
using namespace boost::algorithm;

namespace {
    constexpr int GPS_COMM_TIMEOUT_MS       = 1300;
    constexpr int GPS_NMEA_NORMAL_FRESHNESS = 1000;
    constexpr int GPS_SERVO_FRESHNESS       = 1000;
    constexpr int GPS_LOCK_FRESHNESS        = 2500;
    constexpr int GPS_TIMEOUT_DELAY_MS      = 200;
    constexpr int GPSDO_COMMAND_DELAY_MS    = 200;
}

/*!
 * A NMEA and UBX Parser for the LEA-M8F and other GPSDOs
 */
class gps_ctrl_parser {
  private:
    std::deque<char> gps_data_input;

    std::string parse_ubx() {
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

    std::string parse_nmea() {
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
      void push_data(InputIterator first, InputIterator last) {
        gps_data_input.insert(gps_data_input.end(), first, last);
      }

    std::string get_next_message() {
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


/*!
 * A control for GPSDO devices
 */

gps_ctrl::~gps_ctrl(void){
    /* NOP */
}

class gps_ctrl_impl : public gps_ctrl{
private:
    std::map<std::string, boost::tuple<std::string, boost::system_time, bool> > sentences;
    boost::mutex cache_mutex;
    boost::system_time _last_cache_update;

    gps_ctrl_parser _gps_parser;

    std::string get_sentence(const std::string which, const int max_age_ms, const int timeout, const bool wait_for_next = false)
    {
        std::string sentence;
        boost::system_time now = boost::get_system_time();
        boost::system_time exit_time = now + milliseconds(timeout);
        boost::posix_time::time_duration age;

        if (wait_for_next)
        {
            boost::lock_guard<boost::mutex> lock(cache_mutex);
            update_cache();
            //mark sentence as touched
            if (sentences.find(which) != sentences.end())
                sentences[which].get<2>() = true;
        }
        while (1)
        {
            try
            {
                boost::lock_guard<boost::mutex> lock(cache_mutex);

                // update cache if older than a millisecond
                if (now - _last_cache_update > milliseconds(1))
                {
                    update_cache();
                }

                if (sentences.find(which) == sentences.end())
                {
                    age = milliseconds(max_age_ms);
                } else {
                    age = boost::get_system_time() - sentences[which].get<1>();
                }
                if (age < milliseconds(max_age_ms) and (not (wait_for_next and sentences[which].get<2>())))
                {
                    sentence = sentences[which].get<0>();
                    sentences[which].get<2>() = true;
                }
            } catch(std::exception &e) {
                UHD_LOGGER_DEBUG("GPS") << "get_sentence: " << e.what();
            }

            if (not sentence.empty() or now > exit_time)
            {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            now = boost::get_system_time();
        }

        if (sentence.empty())
        {
            throw uhd::value_error("gps ctrl: No " + which + " message found");
        }

        return sentence;
    }

    static bool is_nmea_checksum_ok(std::string nmea)
    {
        if (nmea.length() < 5 || nmea[0] != '$' || nmea[nmea.length()-3] != '*')
            return false;

        std::stringstream ss;
        uint32_t string_crc;
        uint32_t calculated_crc = 0;

        // get crc from string
        ss << std::hex << nmea.substr(nmea.length()-2, 2);
        ss >> string_crc;

        // calculate crc
        for (size_t i = 1; i < nmea.length()-3; i++)
            calculated_crc ^= nmea[i];

        // return comparison
        return (string_crc == calculated_crc);
    }

  void update_cache() {
    if(not gps_detected()) {
        return;
    }

    std::list<std::string> keys;
    std::map<std::string,std::string> msgs;

    if (_gps_type == GPS_TYPE_LEA_M8F) {
      keys = {"GNGGA", "GNRMC", "TIMELOCK", "DISCSRC"};

      // Concatenate all incoming data into the deque
      for (std::string msg = _recv(); msg.length() > 0; msg = _recv())
      {
        _gps_parser.push_data(msg.begin(), msg.end());
      }

      // Get all GPSDO messages available
      // Creating a map here because we only want the latest of each message type
      for (std::string msg = _gps_parser.get_next_message();
          not msg.empty();
          msg = _gps_parser.get_next_message())
      {
        /*
        if (msg[0] != '$') {
          std::stringstream ss;
          ss << "Got message ";
          for (size_t m = 0; m < msg.size(); m++) {
            ss << std::hex << (unsigned int)(unsigned char)msg[m] << " " << std::dec;
          }
          UHD_LOGGER_WARNING("GPS") << ss.str();
        } // */

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
          uint8_t discSrc    = (flags >> 8) & 0x07;

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

          switch (discSrc) {
              case 0: msgs["DISCSRC"] = "internal"; break;
              case 1: msgs["DISCSRC"] = "gnss"; break;
              default: msgs["DISCSRC"] = "other"; break;
          }
        }
        else if (msg[0] == '\xb5' and msg[1] == '\x62') { /* Ignore unsupported UBX message */ }
        else {
          std::stringstream ss;
          ss << "Unknown message ";
          for (size_t m = 0; m < msg.size(); m++) {
            ss << std::hex << (unsigned int)(unsigned char)msg[m] << " " << std::dec;
          }
          UHD_LOGGER_WARNING("GPS") << ss.str() << ":" << msg << std::endl;
         }
      }
    }
    else {
      keys = {"GPGGA", "GPRMC", "SERVO"};
      static const boost::regex servo_regex("^\\d\\d-\\d\\d-\\d\\d.*$");
      static const boost::regex gp_msg_regex("^\\$GP.*,\\*[0-9A-F]{2}$");

      // Get all GPSDO messages available
      // Creating a map here because we only want the latest of each message type
      for (std::string msg = _recv(0); not msg.empty(); msg = _recv(0))
      {
        // Strip any end of line characters
        erase_all(msg, "\r");
        erase_all(msg, "\n");

        if (msg.empty())
        {
          // Ignore empty strings
          continue;
        }

        if (msg.length() < 6)
        {
            UHD_LOGGER_WARNING("GPS") << __FUNCTION__ << ": Short GPSDO string: " << msg ;
            continue;
        }

        // Look for SERVO message
        if (boost::regex_search(msg, servo_regex, boost::regex_constants::match_continuous))
        {
          msgs["SERVO"] = msg;
        }
        else if (boost::regex_match(msg, gp_msg_regex) and is_nmea_checksum_ok(msg))
        {
          msgs[msg.substr(1,5)] = msg;
        }
        else
        {
            UHD_LOGGER_WARNING("GPS") << __FUNCTION__ << ": Malformed GPSDO string: " << msg ;
        }
      }
    }

    boost::system_time time = boost::get_system_time();

    // Update sentences with newly read data
    for(std::string key:  keys)
    {
      if (not msgs[key].empty())
      {
        sentences[key] = boost::make_tuple(msgs[key], time, false);
      }
    }

    _last_cache_update = time;
  }

public:
  gps_ctrl_impl(uart_iface::sptr uart) :
      _uart(uart),
      _gps_type(GPS_TYPE_NONE)
  {

    std::string reply;
    bool i_heard_some_nmea = false, i_heard_something_weird = false;

    //first we look for an internal GPSDO
    _flush(); //get whatever junk is in the rx buffer right now, and throw it away

    _send("*IDN?\r\n"); //request identity from the GPSDO
    _send("\xB5\x62\x0A\x04\x00\x00\x0E\x34"); // poll UBX-MON-VER

    //then we loop until we either timeout, or until we get a response that indicates we're a JL device
    //maximum response time was measured at ~320ms, so we set the timeout at 650ms
    // For the LEA-M8F, increase the timeout to over one second to increase detection likelihood.
    const boost::system_time comm_timeout = boost::get_system_time() + milliseconds(1200);
    while(boost::get_system_time() < comm_timeout) {
      reply = _recv();
      //known devices are JL "FireFly", "GPSTCXO", and "LC_XO"
      if(reply.find("FireFly") != std::string::npos
         or reply.find("LC_XO") != std::string::npos
         or reply.find("GPSTCXO") != std::string::npos) {
        _gps_type = GPS_TYPE_INTERNAL_GPSDO;
        break;
      } else if(reply.substr(0, 3) == "$GP") {
          i_heard_some_nmea = true; //but keep looking
      } else if(reply.substr(0, 3) == "$GN"
          or reply.substr(0, 2) == "\xB5""\x62") {
          // The u-blox LEA-M8F outputs UBX protocol messages
          _gps_type = GPS_TYPE_LEA_M8F;
          break;
      } else if(not reply.empty()) {
          // wrong baud rate or firmware still initializing
          i_heard_something_weird = true;
          _send("*IDN?\r\n");   //re-send identity request
          _send("\xB5\x62\x0A\x04\x00\x00\x0E\x34"); // poll UBX-MON-VER
      } else {
          // _recv timed out
          _send("*IDN?\r\n");   //re-send identity request
          _send("\xB5\x62\x0A\x04\x00\x00\x0E\x34"); // poll UBX-MON-VER
      }
    }

    if (_gps_type == GPS_TYPE_NONE)
    {
        if(i_heard_some_nmea) {
            _gps_type = GPS_TYPE_GENERIC_NMEA;
        } else if(i_heard_something_weird) {
            UHD_LOGGER_ERROR("GPS") << "GPS invalid reply \"" << reply << "\", assuming none available";
        }
    }

    switch(_gps_type) {
    case GPS_TYPE_INTERNAL_GPSDO:
      erase_all(reply, "\r");
      erase_all(reply, "\n");
      UHD_LOGGER_INFO("GPS") << "Found an internal GPSDO: " << reply;
      init_gpsdo();
      break;

    case GPS_TYPE_GENERIC_NMEA:
        UHD_LOGGER_INFO("GPS") << "Found a generic NMEA GPS device";
      break;

    case GPS_TYPE_LEA_M8F:
      UHD_LOGGER_INFO("GPS") << "Found a LEA-M8F GPS device";
      init_lea_m8f();
      break;

    case GPS_TYPE_NONE:
    default:
        UHD_LOGGER_INFO("GPS") << "No GPSDO found";
      break;

    }

    // initialize cache
    update_cache();
  }

  ~gps_ctrl_impl(void){
    /* NOP */
  }

  //return a list of supported sensors
  std::vector<std::string> get_sensors(void) {
      std::vector<std::string> ret{
          "gps_gpgga",
          "gps_gprmc",
          "gps_gngga",
          "gps_gnrmc",
          "gps_time",
          "gps_locked",
          "gps_servo",
          "gps_timelock",
          "gps_discsrc"
      };
      return ret;
  }

  uhd::sensor_value_t get_sensor(std::string key) {
    if(key == "gps_gpgga"
    or key == "gps_gprmc"
    or key == "gps_gngga"
    or key == "gps_gnrmc"
    or key == "gps_timelock"
    or key == "gps_discsrc") {
        return sensor_value_t(
                 boost::to_upper_copy(key),
                 get_sentence(boost::to_upper_copy(key.substr(4,8)), GPS_NMEA_NORMAL_FRESHNESS, GPS_TIMEOUT_DELAY_MS),
                 "");
    }
    else if(key == "gps_time") {
        return sensor_value_t("GPS epoch time", int(get_epoch_time()), "seconds");
    }
    else if(key == "gps_locked") {
        return sensor_value_t("GPS lock status", locked(), "locked", "unlocked");
    }
    else if(key == "gps_servo") {
        return sensor_value_t(
                 boost::to_upper_copy(key),
                 get_sentence(boost::to_upper_copy(key.substr(4,8)), GPS_SERVO_FRESHNESS, GPS_TIMEOUT_DELAY_MS),
                 "");
    }
    else {
        throw uhd::value_error("gps ctrl get_sensor unknown key: " + key);
    }
  }

private:
  void init_gpsdo(void) {
      //issue some setup stuff so it spits out the appropriate data
      //none of these should issue replies so we don't bother looking for them
      //we have to sleep between commands because the JL device, despite not
      //acking, takes considerable time to process each command.
      const std::vector<std::string> init_cmds = {
          "SYST:COMM:SER:ECHO OFF\r\n",
          "SYST:COMM:SER:PRO OFF\r\n",
          "GPS:GPGGA 1\r\n",
          "GPS:GGAST 0\r\n",
          "GPS:GPRMC 1\r\n",
          "SERV:TRAC 1\r\n"
      };

      for (const auto& cmd : init_cmds) {
          _send(cmd);
          std::this_thread::sleep_for(
                std::chrono::milliseconds(GPSDO_COMMAND_DELAY_MS));
      }
  }

  void init_lea_m8f(void) {
    // Enable the UBX-TIM-TOS and the $GNRMC messages
    const uint8_t en_tim_tos[11] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0d, 0x12, 0x01, 0x2a, 0x8b};
    _send(std::string(reinterpret_cast<const char *>(en_tim_tos), sizeof(en_tim_tos)));

    const uint8_t en_gnrmc[11]   = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x04, 0x01, 0xff, 0x18};
    _send(std::string(reinterpret_cast<const char *>(en_gnrmc), sizeof(en_gnrmc)));
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
    int error_cnt = 0;
    ptime gps_time;
    const std::string rmc = (_gps_type == GPS_TYPE_LEA_M8F) ? "GNRMC" : "GPRMC";
    while(error_cnt < 2) {
        try {
            // wait for next GPRMC string
            std::string reply = get_sentence(rmc, GPS_NMEA_NORMAL_FRESHNESS, GPS_COMM_TIMEOUT_MS, true);

            std::string datestr = get_token(reply, 9);
            std::string timestr = get_token(reply, 1);

            if(datestr.size() == 0 or timestr.size() == 0) {
                throw uhd::value_error(str(boost::format("Invalid response \"%s\"") % reply));
            }

            struct tm raw_date;
            raw_date.tm_year = std::stoi(datestr.substr(4, 2)) + 2000 - 1900; // years since 1900
            raw_date.tm_mon = std::stoi(datestr.substr(2, 2)) - 1; // months since january (0-11)
            raw_date.tm_mday = std::stoi(datestr.substr(0, 2)); // dom (1-31)
            raw_date.tm_hour = std::stoi(timestr.substr(0, 2));
            raw_date.tm_min = std::stoi(timestr.substr(2, 2));
            raw_date.tm_sec = std::stoi(timestr.substr(4,2));
            gps_time = boost::posix_time::ptime_from_tm(raw_date);

            UHD_LOG_TRACE("GPS", "GPS time: " + boost::posix_time::to_simple_string(gps_time));
            return gps_time;

        } catch(std::exception &e) {
            UHD_LOGGER_DEBUG("GPS") << "get_time: " << e.what();
            error_cnt++;
        }
    }
    throw uhd::value_error("get_time: Timeout after no valid message found");

    return gps_time; //keep gcc from complaining
  }

  int64_t get_epoch_time(void) {
      return (get_time() - from_time_t(0)).total_seconds();
  }

  bool gps_detected(void) {
    return (_gps_type != GPS_TYPE_NONE);
  }

  int gps_refclock_frequency(void) {
    if (_gps_type == GPS_TYPE_LEA_M8F) {
      return 30720;
    }
    else if (_gps_type != GPS_TYPE_NONE) {
      return 10000;
    }
    return 0;
  }

  bool locked(void) {
    int error_cnt = 0;
    const std::string locksentence = (_gps_type == GPS_TYPE_LEA_M8F) ? "TIMELOCK" : "GPGGA";
    while(error_cnt < 3) {
        try {
            std::string reply = get_sentence(locksentence, GPS_LOCK_FRESHNESS, GPS_COMM_TIMEOUT_MS);
            if(reply.empty())
                error_cnt++;
            else {
              if (_gps_type == GPS_TYPE_LEA_M8F) {
                return reply == "TIME LOCKED";
              }
              else {
                return (get_token(reply, 6) != "0");
              }
            }
        } catch(std::exception &e) {
            UHD_LOGGER_DEBUG("GPS") << "locked: " << e.what();
            error_cnt++;
        }
    }
    throw uhd::value_error("locked(): unable to determine GPS lock status");
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
    GPS_TYPE_GENERIC_NMEA,
    GPS_TYPE_LEA_M8F,
    GPS_TYPE_NONE
  } _gps_type;

};

/***********************************************************************
 * Public make function for the GPS control
 **********************************************************************/
gps_ctrl::sptr gps_ctrl::make(uart_iface::sptr uart){
    return sptr(new gps_ctrl_impl(uart));
}
