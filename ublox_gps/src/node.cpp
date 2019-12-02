//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#include <cmath>
#include <functional>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/fts_product.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/node.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>

namespace ublox_node {

/**
 * @brief Determine dynamic model from human-readable string.
 * @param model One of the following (case-insensitive):
 *  - portable
 *  - stationary
 *  - pedestrian
 *  - automotive
 *  - sea
 *  - airborne1
 *  - airborne2
 *  - airborne4
 *  - wristwatch
 * @return DynamicModel
 * @throws std::runtime_error on invalid argument.
 */
uint8_t modelFromString(const std::string& model) {
  std::string lower = model;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "portable") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_PORTABLE;
  } else if (lower == "stationary") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_STATIONARY;
  } else if (lower == "pedestrian") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_PEDESTRIAN;
  } else if (lower == "automotive") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AUTOMOTIVE;
  } else if (lower == "sea") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_SEA;
  } else if (lower == "airborne1") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_1G;
  } else if (lower == "airborne2") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_2G;
  } else if (lower == "airborne4") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_4G;
  } else if (lower == "wristwatch") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_WRIST_WATCH;
  }

  throw std::runtime_error("Invalid settings: " + lower +
                           " is not a valid dynamic model.");
}

/**
 * @brief Determine fix mode from human-readable string.
 * @param mode One of the following (case-insensitive):
 *  - 2d
 *  - 3d
 *  - auto
 * @return FixMode
 * @throws std::runtime_error on invalid argument.
 */
uint8_t fixModeFromString(const std::string& mode) {
  std::string lower = mode;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "2d") {
    return ublox_msgs::CfgNAV5::FIX_MODE_2D_ONLY;
  } else if (lower == "3d") {
    return ublox_msgs::CfgNAV5::FIX_MODE_3D_ONLY;
  } else if (lower == "auto") {
    return ublox_msgs::CfgNAV5::FIX_MODE_AUTO;
  }

  throw std::runtime_error("Invalid settings: " + mode +
                           " is not a valid fix mode.");
}

std::vector<std::string> stringSplit(const std::string &str,
                                     const std::string &splitter) {
  std::vector<std::string> ret;
  size_t next = 0;
  size_t current = next;

  if (splitter.empty()) {
    // If the splitter is blank, just return the original
    ret.push_back(str);
    return ret;
  }

  while (next != std::string::npos) {
    next = str.find(splitter, current);
    ret.push_back(str.substr(current, next - current));
    current = next + splitter.length();
  }

  return ret;
}

//
// u-blox ROS Node
//
UbloxNode::UbloxNode() {
  int debug;
  nh->param("debug", debug, 1);
  gps_ = std::make_shared<ublox_gps::Gps>(debug);

  gnss_ = std::make_shared<Gnss>();

  nav_status_pub_ = nh->advertise<ublox_msgs::NavSTATUS>("navstatus", 1);
  nav_posecef_pub_ = nh->advertise<ublox_msgs::NavPOSECEF>("navposecef", 1);
  nav_clock_pub_ = nh->advertise<ublox_msgs::NavCLOCK>("navclock", 1);
  aid_alm_pub_ = nh->advertise<ublox_msgs::AidALM>("aidalm", 1);
  aid_eph_pub_ = nh->advertise<ublox_msgs::AidEPH>("aideph", 1);
  aid_hui_pub_ = nh->advertise<ublox_msgs::AidHUI>("aidhui", 1);

  updater_ = std::make_shared<diagnostic_updater::Updater>();
  updater_->setHardwareID("ublox");

  // configure diagnostic updater for frequency
  freq_diag_ = std::make_shared<FixDiagnostic>(std::string("fix"), kFixFreqTol,
                                               kFixFreqWindow, kTimeStampStatusMin, nav_rate_, meas_rate_, updater_);

  initialize();
}

void UbloxNode::addFirmwareInterface() {
  int ublox_version;
  if (protocol_version_ < 14) {
    components_.push_back(std::make_shared<UbloxFirmware6>(frame_id_, updater_, freq_diag_, gnss_));
    ublox_version = 6;
  } else if (protocol_version_ >= 14 && protocol_version_ <= 15) {
    components_.push_back(std::make_shared<UbloxFirmware7>(frame_id_, updater_, freq_diag_, gnss_));
    ublox_version = 7;
  } else if (protocol_version_ > 15 && protocol_version_ <= 23) {
    components_.push_back(std::make_shared<UbloxFirmware8>(frame_id_, updater_, freq_diag_, gnss_));
    ublox_version = 8;
  } else {
    components_.push_back(std::make_shared<UbloxFirmware9>(frame_id_, updater_, freq_diag_, gnss_));
    ublox_version = 9;
  }

  ROS_INFO("U-Blox Firmware Version: %d", ublox_version);
}


void UbloxNode::addProductInterface(const std::string & product_category,
                                    const std::string & ref_rov) {
  if (product_category.compare("HPG") == 0 && ref_rov.compare("REF") == 0) {
    components_.push_back(std::make_shared<HpgRefProduct>(nav_rate_, meas_rate_, updater_, rtcms_));
  } else if (product_category.compare("HPG") == 0 && ref_rov.compare("ROV") == 0) {
    components_.push_back(std::make_shared<HpgRovProduct>(nav_rate_, updater_));
  } else if (product_category.compare("HPG") == 0) {
    components_.push_back(std::make_shared<HpPosRecProduct>(nav_rate_, meas_rate_, frame_id_, updater_, rtcms_));
  } else if (product_category.compare("TIM") == 0) {
    components_.push_back(std::make_shared<TimProduct>(frame_id_, updater_));
  } else if (product_category.compare("ADR") == 0 ||
             product_category.compare("UDR") == 0) {
    components_.push_back(std::make_shared<AdrUdrProduct>(nav_rate_, meas_rate_, frame_id_, updater_));
  } else if (product_category.compare("FTS") == 0) {
    components_.push_back(std::make_shared<FtsProduct>());
  } else if (product_category.compare("SPG") != 0) {
    ROS_WARN("Product category %s %s from MonVER message not recognized %s",
             product_category.c_str(), ref_rov.c_str(),
             "options are HPG REF, HPG ROV, HPG #.#, TIM, ADR, UDR, FTS, SPG");
  }
}

void UbloxNode::getRosParams() {
  nh->param("device", device_, std::string("/dev/ttyACM0"));
  nh->param("frame_id", frame_id_, std::string("gps"));

  // Save configuration parameters
  getRosUint("load/mask", load_.load_mask, 0);
  getRosUint("load/device", load_.device_mask, 0);
  getRosUint("save/mask", save_.save_mask, 0);
  getRosUint("save/device", save_.device_mask, 0);

  // UART 1 params
  getRosUint("uart1/baudrate", baudrate_, 9600);
  getRosUint("uart1/in", uart_in_, ublox_msgs::CfgPRT::PROTO_UBX
                                    | ublox_msgs::CfgPRT::PROTO_NMEA
                                    | ublox_msgs::CfgPRT::PROTO_RTCM);
  getRosUint("uart1/out", uart_out_, ublox_msgs::CfgPRT::PROTO_UBX);
  // USB params
  set_usb_ = false;
  if (nh->hasParam("usb/in") || nh->hasParam("usb/out")) {
    set_usb_ = true;
    if (!getRosUint("usb/in", usb_in_)) {
      throw std::runtime_error(std::string("usb/out is set, therefore ") +
        "usb/in must be set");
    }
    if (!getRosUint("usb/out", usb_out_)) {
      throw std::runtime_error(std::string("usb/in is set, therefore ") +
        "usb/out must be set");
    }
    getRosUint("usb/tx_ready", usb_tx_, 0);
  }
  // Measurement rate params
  nh->param("rate", rate_, 4.0);  // in Hz
  getRosUint("nav_rate", nav_rate_, 1);  // # of measurement rate cycles
  // RTCM params
  std::vector<uint8_t> rtcm_ids;
  std::vector<uint8_t> rtcm_rates;
  getRosUint("rtcm/ids", rtcm_ids);  // RTCM output message IDs
  getRosUint("rtcm/rates", rtcm_rates);  // RTCM output message rates
  // PPP: Advanced Setting
  declareRosBoolean("enable_ppp", false);
  // SBAS params, only for some devices
  declareRosBoolean("gnss/sbas", false);
  declareRosBoolean("gnss/gps", true);
  declareRosBoolean("gnss/glonass", false);
  declareRosBoolean("gnss/qzss", false);
  declareRosBoolean("gnss/galileo", false);
  declareRosBoolean("gnss/beidou", false);
  declareRosBoolean("gnss/imes", false);
  getRosUint("sbas/max", max_sbas_, 0); // Maximum number of SBAS channels
  getRosUint("sbas/usage", sbas_usage_, 0);
  nh->param("dynamic_model", dynamic_model_, std::string("portable"));
  nh->param("fix_mode", fix_mode_, std::string("auto"));
  getRosUint("dr_limit", dr_limit_, 0); // Dead reckoning limit

  if (getRosBoolean("enable_ppp")) {
    ROS_WARN("Warning: PPP is enabled - this is an expert setting.");
  }

  checkMin(rate_, 0, "rate");

  if (rtcm_ids.size() != rtcm_rates.size()) {
    throw std::runtime_error(std::string("Invalid settings: size of rtcm_ids") +
                             " must match size of rtcm_rates");
  }

  rtcms_.resize(rtcm_ids.size());
  for (size_t i = 0; i < rtcm_ids.size(); ++i) {
    rtcms_[i].id = rtcm_ids[i];
    rtcms_[i].rate = rtcm_rates[i];
  }

  dmodel_ = modelFromString(dynamic_model_);
  fmode_ = fixModeFromString(fix_mode_);

  declareRosBoolean("dat/set", false);
  if (getRosBoolean("dat/set")) {
    std::vector<float> shift, rot;
    if (!nh->getParam("dat/majA", cfg_dat_.maj_a)
        || nh->getParam("dat/flat", cfg_dat_.flat)
        || nh->getParam("dat/shift", shift)
        || nh->getParam("dat/rot", rot)
        || nh->getParam("dat/scale", cfg_dat_.scale)) {
      throw std::runtime_error(std::string("dat/set is true, therefore ") +
         "dat/majA, dat/flat, dat/shift, dat/rot, & dat/scale must be set");
    }
    if (shift.size() != 3 || rot.size() != 3) {
      throw std::runtime_error(std::string("size of dat/shift & dat/rot ") +
                               "must be 3");
    }
    checkRange(cfg_dat_.maj_a, 6300000.0, 6500000.0, "dat/majA");
    checkRange(cfg_dat_.flat, 0.0, 500.0, "dat/flat");

    checkRange(shift, 0.0, 500.0, "dat/shift");
    cfg_dat_.d_x = shift[0];
    cfg_dat_.d_y = shift[1];
    cfg_dat_.d_z = shift[2];

    checkRange(rot, -5000.0, 5000.0, "dat/rot");
    cfg_dat_.rot_x = rot[0];
    cfg_dat_.rot_y = rot[1];
    cfg_dat_.rot_z = rot[2];

    checkRange(cfg_dat_.scale, 0.0, 50.0, "scale");
  }

  // measurement period [ms]
  meas_rate_ = 1000 / rate_;

  // activate/deactivate any config
  declareRosBoolean("config_on_startup", true);
  declareRosBoolean("raw_data", false);
  declareRosBoolean("clear_bbr", false);
  declareRosBoolean("save_on_shutdown", false);

  // raw data stream logging
  rawDataStreamPa_.getRosParams();

  // NMEA parameters
  declareRosBoolean("nmea/set", false);
  declareRosBoolean("nmea/compat", false);
  declareRosBoolean("nmea/consider", false);
  declareRosBoolean("nmea/limit82", false);
  declareRosBoolean("nmea/high_prec", false);
  declareRosBoolean("nmea/filter/pos", false);
  declareRosBoolean("nmea/filter/msk_pos", false);
  declareRosBoolean("nmea/filter/time", false);
  declareRosBoolean("nmea/filter/date", false);
  declareRosBoolean("nmea/filter/sbas", false);
  declareRosBoolean("nmea/filter/track", false);
  declareRosBoolean("nmea/filter/gps_only", false);
  declareRosBoolean("nmea/gnssToFilter/gps", false);
  declareRosBoolean("nmea/gnssToFilter/sbas", false);
  declareRosBoolean("nmea/gnssToFilter/qzss", false);
  declareRosBoolean("nmea/gnssToFilter/glonass", false);
  declareRosBoolean("nmea/gnssToFilter/beidou", false);

  // Publish parameters
  declareRosBoolean("publish/all", false);

  declareRosBoolean("publish/nav/all", getRosBoolean("publish/all"));
  declareRosBoolean("publish/nav/att", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/clock", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/heading", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/posecef", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/posllh", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/pvt", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/relposned", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/sat", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/sol", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/svin", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/svinfo", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/status", getRosBoolean("publish/nav/all"));
  declareRosBoolean("publish/nav/velned", getRosBoolean("publish/nav/all"));

  declareRosBoolean("publish/rxm/all", getRosBoolean("publish/all"));
  declareRosBoolean("publish/rxm/almRaw", getRosBoolean("publish/rxm/all"));
  declareRosBoolean("publish/rxm/eph", getRosBoolean("publish/rxm/all"));
  declareRosBoolean("publish/rxm/rtcm", getRosBoolean("publish/rxm/all"));
  declareRosBoolean("publish/rxm/raw", getRosBoolean("publish/rxm/all"));
  declareRosBoolean("publish/rxm/sfrb", getRosBoolean("publish/rxm/all"));

  declareRosBoolean("publish/aid/all", getRosBoolean("publish/all"));
  declareRosBoolean("publish/aid/alm", getRosBoolean("publish/aid/all"));
  declareRosBoolean("publish/aid/eph", getRosBoolean("publish/aid/all"));
  declareRosBoolean("publish/aid/hui", getRosBoolean("publish/aid/all"));

  declareRosBoolean("publish/mon/all", getRosBoolean("publish/all"));
  declareRosBoolean("publish/mon/hw", getRosBoolean("publish/mon/all"));

  declareRosBoolean("publish/tim/tm2", false);

  // INF parameters
  declareRosBoolean("inf/all", true);
  declareRosBoolean("inf/debug", false);
  declareRosBoolean("inf/error", getRosBoolean("inf/all"));
  declareRosBoolean("inf/notice", getRosBoolean("inf/all"));
  declareRosBoolean("inf/test", getRosBoolean("inf/all"));
  declareRosBoolean("inf/warning", getRosBoolean("inf/all"));

  // ESF parameters
  declareRosBoolean("publish/esf/all", true);
  declareRosBoolean("publish/esf/ins", getRosBoolean("publish/esf/all"));
  declareRosBoolean("publish/esf/meas", getRosBoolean("publish/esf/all"));
  declareRosBoolean("publish/esf/raw", getRosBoolean("publish/esf/all"));
  declareRosBoolean("publish/esf/status", getRosBoolean("publish/esf/all"));

  // HNR parameters
  declareRosBoolean("publish/hnr/pvt", true);
}

void UbloxNode::pollMessages(const ros::TimerEvent& event) {
  static std::vector<uint8_t> payload(1, 1);
  if (getRosBoolean("publish/aid/alm")) {
    gps_->poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM, payload);
  }
  if (getRosBoolean("publish/aid/eph")) {
    gps_->poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH, payload);
  }
  if (getRosBoolean("publish/aid/hui")) {
    gps_->poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI);
  }

  payload[0]++;
  if (payload[0] > 32) {
    payload[0] = 1;
  }
}

void UbloxNode::printInf(const ublox_msgs::Inf &m, uint8_t id) {
  if (id == ublox_msgs::Message::INF::ERROR) {
    ROS_ERROR_STREAM("INF: " << std::string(m.str.begin(), m.str.end()));
  } else if (id == ublox_msgs::Message::INF::WARNING) {
    ROS_WARN_STREAM("INF: " << std::string(m.str.begin(), m.str.end()));
  } else if (id == ublox_msgs::Message::INF::DEBUG) {
    ROS_DEBUG_STREAM("INF: " << std::string(m.str.begin(), m.str.end()));
  } else {
    ROS_INFO_STREAM("INF: " << std::string(m.str.begin(), m.str.end()));
  }
}

void UbloxNode::subscribe() {
  ROS_DEBUG("Subscribing to U-Blox messages");
  // subscribe messages

  // Nav Messages
  if (getRosBoolean("publish/nav/status")) {
    gps_->subscribe<ublox_msgs::NavSTATUS>([this](const ublox_msgs::NavSTATUS &m) { nav_status_pub_.publish(m); },
                                           1);
  }

  if (getRosBoolean("publish/nav/posecef")) {
    gps_->subscribe<ublox_msgs::NavPOSECEF>([this](const ublox_msgs::NavPOSECEF &m) { nav_posecef_pub_.publish(m); },
                                            1);
  }

  if (getRosBoolean("publish/nav/clock")) {
    gps_->subscribe<ublox_msgs::NavCLOCK>([this](const ublox_msgs::NavCLOCK &m) { nav_clock_pub_.publish(m); },
                                          1);
  }

  // INF messages
  if (getRosBoolean("inf/debug")) {
    gps_->subscribeId<ublox_msgs::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::DEBUG),
        ublox_msgs::Message::INF::DEBUG);
  }

  if (getRosBoolean("inf/error")) {
    gps_->subscribeId<ublox_msgs::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::ERROR),
        ublox_msgs::Message::INF::ERROR);
  }

  if (getRosBoolean("inf/notice")) {
    gps_->subscribeId<ublox_msgs::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::NOTICE),
        ublox_msgs::Message::INF::NOTICE);
  }

  if (getRosBoolean("inf/test")) {
    gps_->subscribeId<ublox_msgs::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::TEST),
        ublox_msgs::Message::INF::TEST);
  }

  if (getRosBoolean("inf/warning")) {
    gps_->subscribeId<ublox_msgs::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::WARNING),
        ublox_msgs::Message::INF::WARNING);
  }

  // AID messages
  if (getRosBoolean("publish/aid/alm")) {
    gps_->subscribe<ublox_msgs::AidALM>([this](const ublox_msgs::AidALM &m) { aid_alm_pub_.publish(m); },
                                        1);
  }

  if (getRosBoolean("publish/aid/eph")) {
    gps_->subscribe<ublox_msgs::AidEPH>([this](const ublox_msgs::AidEPH &m) { aid_eph_pub_.publish(m); },
                                        1);
  }

  if (getRosBoolean("publish/aid/hui")) {
    gps_->subscribe<ublox_msgs::AidHUI>([this](const ublox_msgs::AidHUI &m) { aid_hui_pub_.publish(m); },
                                        1);
  }

  for (size_t i = 0; i < components_.size(); i++) {
    components_[i]->subscribe(gps_);
  }
}

void UbloxNode::initializeRosDiagnostics() {
  declareRosBoolean("diagnostic_period", kDiagnosticPeriod);

  for (int i = 0; i < components_.size(); i++) {
    components_[i]->initializeRosDiagnostics();
  }
}

void UbloxNode::processMonVer() {
  ublox_msgs::MonVER monVer;
  if (!gps_->poll(monVer)) {
    throw std::runtime_error("Failed to poll MonVER & set relevant settings");
  }

  ROS_DEBUG("%s, HW VER: %s", monVer.sw_version.c_array(),
            monVer.hw_version.c_array());
  // Convert extension to vector of strings
  std::vector<std::string> extension;
  extension.reserve(monVer.extension.size());
  for (std::size_t i = 0; i < monVer.extension.size(); ++i) {
    ROS_DEBUG("%s", monVer.extension[i].field.c_array());
    // Find the end of the string (null character)
    unsigned char* end = std::find(monVer.extension[i].field.begin(),
          monVer.extension[i].field.end(), '\0');
    extension.push_back(std::string(monVer.extension[i].field.begin(), end));
  }

  // Get the protocol version
  for (std::size_t i = 0; i < extension.size(); ++i) {
    std::size_t found = extension[i].find("PROTVER");
    if (found != std::string::npos) {
      protocol_version_ = ::atof(
          extension[i].substr(8, extension[i].size()-8).c_str());
      break;
    }
  }
  if (protocol_version_ == 0) {
    ROS_WARN("Failed to parse MonVER and determine protocol version. %s",
             "Defaulting to firmware version 6.");
  }
  addFirmwareInterface();

  if (protocol_version_ < 18) {
    // Final line contains supported GNSS delimited by ;
    std::vector<std::string> strs;
    if (extension.size() > 0) {
      strs = stringSplit(extension[extension.size() - 1], ";");
    }
    for (size_t i = 0; i < strs.size(); i++) {
      gnss_->add(strs[i]);
    }
  } else {
    for (std::size_t i = 0; i < extension.size(); ++i) {
      std::vector<std::string> strs;
      // Up to 2nd to last line
      if (i <= extension.size() - 2) {
        strs = stringSplit(extension[i], "=");
        if (strs.size() > 1) {
          if (strs[0].compare(std::string("FWVER")) == 0) {
            if (strs[1].length() > 8) {
              addProductInterface(strs[1].substr(0, 3), strs[1].substr(8, 10));
            } else {
              addProductInterface(strs[1].substr(0, 3));
            }
            continue;
          }
        }
      }
      // Last 1-2 lines contain supported GNSS
      if (i >= extension.size() - 2) {
        strs = stringSplit(extension[i], ";");
        for (size_t i = 0; i < strs.size(); i++) {
          gnss_->add(strs[i]);
        }
      }
    }
  }
}

bool UbloxNode::configureUblox() {
  try {
    if (!gps_->isInitialized()) {
      throw std::runtime_error("Failed to initialize.");
    }
    if (load_.load_mask != 0) {
      ROS_DEBUG("Loading u-blox configuration from memory. %u", load_.load_mask);
      if (!gps_->configure(load_)) {
        throw std::runtime_error(std::string("Failed to load configuration ") +
                                 "from memory");
      }
      if (load_.load_mask & load_.MASK_IO_PORT) {
        ROS_DEBUG("Loaded I/O configuration from memory, resetting serial %s",
          "communications.");
        std::chrono::seconds wait(kResetWait);
        gps_->reset(wait);
        if (!gps_->isConfigured())
          throw std::runtime_error(std::string("Failed to reset serial I/O") +
            "after loading I/O configurations from device memory.");
      }
    }

    if (getRosBoolean("config_on_startup")) {
      if (set_usb_) {
        gps_->configUsb(usb_tx_, usb_in_, usb_out_);
      }
      if (!gps_->configRate(meas_rate_, nav_rate_)) {
        std::stringstream ss;
        ss << "Failed to set measurement rate to " << meas_rate_
          << "ms and navigation rate to " << nav_rate_;
        throw std::runtime_error(ss.str());
      }
      // If device doesn't have SBAS, will receive NACK (causes exception)
      if (gnss_->isSupported("SBAS")) {
        if (!gps_->configSbas(getRosBoolean("gnss/sbas"), sbas_usage_, max_sbas_)) {
          throw std::runtime_error(std::string("Failed to ") +
                                  (getRosBoolean("gnss/sbas") ? "enable" : "disable") +
                                  " SBAS.");
        }
      }
      if (!gps_->setPpp(getRosBoolean("enable_ppp"))) {
        throw std::runtime_error(std::string("Failed to ") +
                                (getRosBoolean("enable_ppp") ? "enable" : "disable")
                                + " PPP.");
      }
      if (!gps_->setDynamicModel(dmodel_)) {
        throw std::runtime_error("Failed to set model: " + dynamic_model_ + ".");
      }
      if (!gps_->setFixMode(fmode_)) {
        throw std::runtime_error("Failed to set fix mode: " + fix_mode_ + ".");
      }
      if (!gps_->setDeadReckonLimit(dr_limit_)) {
        std::stringstream ss;
        ss << "Failed to set dead reckoning limit: " << dr_limit_ << ".";
        throw std::runtime_error(ss.str());
      }
      if (getRosBoolean("dat/set") && !gps_->configure(cfg_dat_)) {
        throw std::runtime_error("Failed to set user-defined datum.");
      }
      // Configure each component
      for (int i = 0; i < components_.size(); i++) {
        if (!components_[i]->configureUblox(gps_)) {
          return false;
        }
      }
    }
    if (save_.save_mask != 0) {
      ROS_DEBUG("Saving the u-blox configuration, mask %u, device %u",
                save_.save_mask, save_.device_mask);
      if (!gps_->configure(save_)) {
        ROS_ERROR("u-blox unable to save configuration to non-volatile memory");
      }
    }
  } catch (std::exception& e) {
    ROS_FATAL("Error configuring u-blox: %s", e.what());
    return false;
  }
  return true;
}

void UbloxNode::configureInf() {
  ublox_msgs::CfgINF msg;
  // Subscribe to UBX INF messages
  ublox_msgs::CfgINFBlock block;
  block.protocol_id = block.PROTOCOL_ID_UBX;
  // Enable desired INF messages on each UBX port
  uint8_t mask = (getRosBoolean("inf/error") ? block.INF_MSG_ERROR : 0) |
                 (getRosBoolean("inf/warning") ? block.INF_MSG_WARNING : 0) |
                 (getRosBoolean("inf/notice") ? block.INF_MSG_NOTICE : 0) |
                 (getRosBoolean("inf/test") ? block.INF_MSG_TEST : 0) |
                 (getRosBoolean("inf/debug") ? block.INF_MSG_DEBUG : 0);
  for (size_t i = 0; i < block.inf_msg_mask.size(); i++) {
    block.inf_msg_mask[i] = mask;
  }

  msg.blocks.push_back(block);

  // IF NMEA is enabled
  if (uart_in_ & ublox_msgs::CfgPRT::PROTO_NMEA) {
    ublox_msgs::CfgINFBlock block;
    block.protocol_id = block.PROTOCOL_ID_NMEA;
    // Enable desired INF messages on each NMEA port
    for (size_t i = 0; i < block.inf_msg_mask.size(); i++) {
      block.inf_msg_mask[i] = mask;
    }
    msg.blocks.push_back(block);
  }

  ROS_DEBUG("Configuring INF messages");
  if (!gps_->configure(msg)) {
    ROS_WARN("Failed to configure INF messages");
  }
}

void UbloxNode::initializeIo() {
  gps_->setConfigOnStartup(getRosBoolean("config_on_startup"));

  std::smatch match;
  if (std::regex_match(device_, match,
                       std::regex("(tcp|udp)://(.+):(\\d+)"))) {
    std::string proto(match[1]);
    if (proto == "tcp") {
      std::string host(match[2]);
      std::string port(match[3]);
      ROS_INFO("Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(),
               port.c_str());
      gps_->initializeTcp(host, port);
    } else {
      throw std::runtime_error("Protocol '" + proto + "' is unsupported");
    }
  } else {
    gps_->initializeSerial(device_, baudrate_, uart_in_, uart_out_);
  }

  // raw data stream logging
  if (rawDataStreamPa_.isEnabled()) {
    gps_->setRawDataCallback(
      std::bind(&RawDataStreamPa::ubloxCallback, &rawDataStreamPa_, std::placeholders::_1, std::placeholders::_2));
    rawDataStreamPa_.initialize();
  }
}

void UbloxNode::initialize() {
  // Params must be set before initializing IO
  getRosParams();
  initializeIo();
  // Must process Mon VER before setting firmware/hardware params
  processMonVer();
  if (protocol_version_ <= 14) {
    if (getRosBoolean("raw_data")) {
      components_.push_back(std::make_shared<RawDataProduct>(nav_rate_, meas_rate_, updater_));
    }
  }
  // Must set firmware & hardware params before initializing diagnostics
  for (size_t i = 0; i < components_.size(); i++) {
    components_[i]->getRosParams();
  }
  // Do this last
  initializeRosDiagnostics();

  if (configureUblox()) {
    ROS_INFO("U-Blox configured successfully.");
    // Subscribe to all U-Blox messages
    subscribe();
    // Configure INF messages (needs INF params, call after subscribing)
    configureInf();

    ros::Timer poller;
    poller = nh->createTimer(ros::Duration(kPollDuration),
                             &UbloxNode::pollMessages,
                             this);
    poller.start();
    ros::spin();
  }
  shutdown();
}

void UbloxNode::shutdown() {
  if (gps_->isInitialized()) {
    gps_->close();
    ROS_INFO("Closed connection to %s.", device_.c_str());
  }
}

//
// U-Blox Firmware (all versions)
//
UbloxFirmware::UbloxFirmware(std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<Gnss> gnss) : updater_(updater), gnss_(gnss)
{
}

void UbloxFirmware::initializeRosDiagnostics() {
  updater_->add("fix", this, &UbloxFirmware::fixDiagnostic);
  updater_->force_update();
}

//
// U-Blox Firmware Version 6
//
UbloxFirmware6::UbloxFirmware6(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss)
  : UbloxFirmware(updater, gnss), frame_id_(frame_id), freq_diag_(freq_diag)
{
  nav_pos_llh_pub_ =
    nh->advertise<ublox_msgs::NavPOSLLH>("navposllh", 1);
  fix_pub_ =
    nh->advertise<sensor_msgs::NavSatFix>("fix", 1);

  nav_vel_ned_pub_ =
    nh->advertise<ublox_msgs::NavVELNED>("navvelned", 1);

  vel_pub_ =
    nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity",
                                                             1);

  nav_sol_pub_ =
    nh->advertise<ublox_msgs::NavSOL>("navsol", 1);

  nav_svinfo_pub_ =
    nh->advertise<ublox_msgs::NavSVINFO>("navinfo", 1);

  mon_hw_pub_ =
    nh->advertise<ublox_msgs::MonHW6>("monhw", 1);
}

void UbloxFirmware6::getRosParams() {
  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::NavSatStatus::SERVICE_GPS;

  if (getRosBoolean("nmea/set")) {
    bool compat, consider;

    if (!getRosUint("nmea/version", cfg_nmea_.version)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/version must be set");
    }
    if (!getRosUint("nmea/num_sv", cfg_nmea_.num_sv)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                "true, therefore nmea/num_sv must be set");
    }

    // set flags
    cfg_nmea_.flags = getRosBoolean("nmea/compat") ? cfg_nmea_.FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= getRosBoolean("nmea/consider") ? cfg_nmea_.FLAGS_CONSIDER : 0;

    // set filter
    bool temp;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/pos") ? cfg_nmea_.FILTER_POS : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/msk_pos") ? cfg_nmea_.FILTER_MSK_POS : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/time") ? cfg_nmea_.FILTER_TIME : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/date") ? cfg_nmea_.FILTER_DATE : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/sbas") ? cfg_nmea_.FILTER_SBAS_FILT : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/track") ? cfg_nmea_.FILTER_TRACK : 0;
  }
}

bool UbloxFirmware6::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  ROS_WARN("ublox_version < 7, ignoring GNSS settings");

  if (getRosBoolean("nmea/set") && !gps->configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware6::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Always subscribes to these messages, but may not publish to ROS topic
  // Subscribe to Nav POSLLH
  gps->subscribe<ublox_msgs::NavPOSLLH>(std::bind(
      &UbloxFirmware6::callbackNavPosLlh, this, std::placeholders::_1), 1);
  gps->subscribe<ublox_msgs::NavSOL>(std::bind(
  // Subscribe to Nav SOL
      &UbloxFirmware6::callbackNavSol, this, std::placeholders::_1), 1);
  // Subscribe to Nav VELNED
  gps->subscribe<ublox_msgs::NavVELNED>(std::bind(
      &UbloxFirmware6::callbackNavVelNed, this, std::placeholders::_1), 1);

  // Subscribe to Nav SVINFO
  if (getRosBoolean("publish/nav/svinfo")) {
    gps->subscribe<ublox_msgs::NavSVINFO>([this](const ublox_msgs::NavSVINFO &m) { nav_svinfo_pub_.publish(m); },
                                          kNavSvInfoSubscribeRate);
  }

  // Subscribe to Mon HW
  if (getRosBoolean("publish/mon/hw")) {
    gps->subscribe<ublox_msgs::MonHW6>([this](const ublox_msgs::MonHW6 &m) { mon_hw_pub_.publish(m); },
                                       1);
  }
}

void UbloxFirmware6::fixDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  // Set the diagnostic level based on the fix status
  if (last_nav_sol_.gps_fix == ublox_msgs::NavSOL::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::NavSOL::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "2D fix";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::NavSOL::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "3D fix";
  } else if (last_nav_sol_.gps_fix ==
             ublox_msgs::NavSOL::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::NavSOL::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "Time fix only";
  }
  // If fix is not ok (within DOP & Accuracy Masks), raise the diagnostic level
  if (!(last_nav_sol_.flags & ublox_msgs::NavSOL::FLAGS_GPS_FIX_OK)) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message += ", fix not ok";
  }
  // Raise diagnostic level to error if no fix
  if (last_nav_sol_.gps_fix == ublox_msgs::NavSOL::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "No fix";
  }

  // Add last fix position
  stat.add("iTOW [ms]", last_nav_pos_.i_tow);
  stat.add("Latitude [deg]", last_nav_pos_.lat * 1e-7);
  stat.add("Longitude [deg]", last_nav_pos_.lon * 1e-7);
  stat.add("Altitude [m]", last_nav_pos_.height * 1e-3);
  stat.add("Height above MSL [m]", last_nav_pos_.h_msl * 1e-3);
  stat.add("Horizontal Accuracy [m]", last_nav_pos_.h_acc * 1e-3);
  stat.add("Vertical Accuracy [m]", last_nav_pos_.v_acc * 1e-3);
  stat.add("# SVs used", (int)last_nav_sol_.num_sv);
}

void UbloxFirmware6::callbackNavPosLlh(const ublox_msgs::NavPOSLLH& m) {
  if (getRosBoolean("publish/nav/posllh")) {
    nav_pos_llh_pub_.publish(m);
  }

  // Position message
  if (m.i_tow == last_nav_vel_.i_tow) {
    fix_.header.stamp = velocity_.header.stamp; // use last timestamp
  } else {
    fix_.header.stamp = ros::Time::now(); // new timestamp
  }

  fix_.header.frame_id = frame_id_;
  fix_.latitude = m.lat * 1e-7;
  fix_.longitude = m.lon * 1e-7;
  fix_.altitude = m.height * 1e-3;

  if (last_nav_sol_.gps_fix >= last_nav_sol_.GPS_2D_FIX) {
    fix_.status.status = fix_.status.STATUS_FIX;
  } else {
    fix_.status.status = fix_.status.STATUS_NO_FIX;
  }

  // Convert from mm to m
  const double var_h = pow(m.h_acc / 1000.0, 2);
  const double var_v = pow(m.v_acc / 1000.0, 2);

  fix_.position_covariance[0] = var_h;
  fix_.position_covariance[4] = var_h;
  fix_.position_covariance[8] = var_v;
  fix_.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix_.status.service = fix_.status.SERVICE_GPS;
  fix_pub_.publish(fix_);
  last_nav_pos_ = m;
  //  update diagnostics
  freq_diag_->diagnostic->tick(fix_.header.stamp);
  updater_->update();
}

void UbloxFirmware6::callbackNavVelNed(const ublox_msgs::NavVELNED& m) {
  if (getRosBoolean("publish/nav/velned")) {
    nav_vel_ned_pub_.publish(m);
  }

  // Example geometry message
  if (m.i_tow == last_nav_pos_.i_tow) {
    velocity_.header.stamp = fix_.header.stamp; // same time as last navposllh
  } else {
    velocity_.header.stamp = ros::Time::now(); // create a new timestamp
  }
  velocity_.header.frame_id = frame_id_;

  //  convert to XYZ linear velocity
  velocity_.twist.twist.linear.x = m.vel_e / 100.0;
  velocity_.twist.twist.linear.y = m.vel_n / 100.0;
  velocity_.twist.twist.linear.z = -m.vel_d / 100.0;

  const double var_speed = pow(m.s_acc / 100.0, 2);

  const int cols = 6;
  velocity_.twist.covariance[cols * 0 + 0] = var_speed;
  velocity_.twist.covariance[cols * 1 + 1] = var_speed;
  velocity_.twist.covariance[cols * 2 + 2] = var_speed;
  velocity_.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

  vel_pub_.publish(velocity_);
  last_nav_vel_ = m;
}

void UbloxFirmware6::callbackNavSol(const ublox_msgs::NavSOL& m) {
  if (getRosBoolean("publish/nav/sol")) {
    nav_sol_pub_.publish(m);
  }
  last_nav_sol_ = m;
}

//
// Ublox Firmware Version 7
//
void UbloxFirmware7::getRosParams() {
  //
  // GNSS configuration
  //
  // GNSS enable/disable
  enable_gps_ = getRosBoolean("gnss/gps");
  enable_glonass_ = getRosBoolean("gnss/glonass");
  enable_qzss_ = getRosBoolean("gnss/qzss");

  getRosUint("gnss/qzss_sig_cfg", qzss_sig_cfg_,
              ublox_msgs::CfgGNSSBlock::SIG_CFG_QZSS_L1CA);

  if (enable_gps_ && !gnss_->isSupported("GPS")) {
    ROS_WARN("gnss/gps is true, but GPS GNSS is not supported by this device");
  }
  if (enable_glonass_ && !gnss_->isSupported("GLO")) {
    ROS_WARN("gnss/glonass is true, but GLONASS is not %s",
             "supported by this device");
  }
  if (enable_qzss_ && !gnss_->isSupported("QZSS")) {
    ROS_WARN("gnss/qzss is true, but QZSS is not supported by this device");
  }
  if (getRosBoolean("gnss/sbas") && !gnss_->isSupported("SBAS")) {
    ROS_WARN("gnss/sbas is true, but SBAS is not supported by this device");
  }

  if (nh->hasParam("gnss/galileo")) {
    ROS_WARN("ublox_version < 8, ignoring Galileo GNSS Settings");
  }
  if (nh->hasParam("gnss/beidou")) {
    ROS_WARN("ublox_version < 8, ignoring BeiDou Settings");
  }
  if (nh->hasParam("gnss/imes")) {
    ROS_WARN("ublox_version < 8, ignoring IMES GNSS Settings");
  }

  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::NavSatStatus::SERVICE_GPS
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GLONASS;

  //
  // NMEA Configuration
  //
  if (getRosBoolean("nmea/set")) {
    bool compat, consider;

    if (!getRosUint("nmea/version", cfg_nmea_.nmea_version)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/version must be set");
    }
    if (!getRosUint("nmea/num_sv", cfg_nmea_.num_sv)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                "true, therefore nmea/num_sv must be set");
    }
    if (!getRosUint("nmea/sv_numbering", cfg_nmea_.sv_numbering)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/sv_numbering must be set");
    }
    if (!nh->getParam("nmea/compat", compat)) {
        throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/compat must be set");
    }
    if (!nh->getParam("nmea/consider", consider)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/consider must be set");
    }

    // set flags
    cfg_nmea_.flags = compat ? cfg_nmea_.FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= consider ? cfg_nmea_.FLAGS_CONSIDER : 0;
    // set filter
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/pos") ? cfg_nmea_.FILTER_POS : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/msk_pos") ? cfg_nmea_.FILTER_MSK_POS : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/time") ? cfg_nmea_.FILTER_TIME : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/date") ? cfg_nmea_.FILTER_DATE : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/gps_only") ? cfg_nmea_.FILTER_GPS_ONLY : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/track") ? cfg_nmea_.FILTER_TRACK : 0;
    // set gnssToFilter
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/gps") ? cfg_nmea_.GNSS_TO_FILTER_GPS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/sbas") ? cfg_nmea_.GNSS_TO_FILTER_SBAS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/qzss") ? cfg_nmea_.GNSS_TO_FILTER_QZSS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/glonass") ? cfg_nmea_.GNSS_TO_FILTER_GLONASS : 0;

    getRosUint("nmea/main_talker_id", cfg_nmea_.main_talker_id);
    getRosUint("nmea/gsv_talker_id", cfg_nmea_.gsv_talker_id);
  }
}

bool UbloxFirmware7::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  /** Configure the GNSS **/
  ublox_msgs::CfgGNSS cfgGNSSRead;
  if (gps->poll(cfgGNSSRead)) {
    ROS_DEBUG("Read GNSS config.");
    ROS_DEBUG("Num. tracking channels in hardware: %i", cfgGNSSRead.num_trk_ch_hw);
    ROS_DEBUG("Num. tracking channels to use: %i", cfgGNSSRead.num_trk_ch_use);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  ublox_msgs::CfgGNSS cfgGNSSWrite;
  cfgGNSSWrite.num_config_blocks = 1;  // do services one by one
  cfgGNSSWrite.num_trk_ch_hw = cfgGNSSRead.num_trk_ch_hw;
  cfgGNSSWrite.num_trk_ch_use = cfgGNSSRead.num_trk_ch_use;
  cfgGNSSWrite.msg_ver = 0;

  // configure GLONASS
  if (gnss_->isSupported("GLO")) {
    ublox_msgs::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_GLONASS;
    block.res_trk_ch = block.RES_TRK_CH_GLONASS;
    block.max_trk_ch = block.MAX_TRK_CH_GLONASS;
    block.flags = enable_glonass_ ? block.SIG_CFG_GLONASS_L1OF : 0;
    cfgGNSSWrite.blocks.push_back(block);
    if (!gps->configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_glonass_) ? "enable" : "disable") +
                               " GLONASS.");
    }
  }

  if (gnss_->isSupported("QZSS")) {
    // configure QZSS
    ublox_msgs::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_QZSS;
    block.res_trk_ch = block.RES_TRK_CH_QZSS;
    block.max_trk_ch = block.MAX_TRK_CH_QZSS;
    block.flags = enable_qzss_ ? qzss_sig_cfg_ : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps->configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_glonass_) ? "enable" : "disable") +
                               " QZSS.");
    }
  }

  if (gnss_->isSupported("SBAS")) {
    // configure SBAS
    ublox_msgs::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_SBAS;
    block.res_trk_ch = block.RES_TRK_CH_SBAS;
    block.max_trk_ch = block.MAX_TRK_CH_SBAS;
    block.flags = getRosBoolean("gnss/sbas") ? block.SIG_CFG_SBAS_L1CA : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps->configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               (getRosBoolean("gnss/sbas") ? "enable" : "disable") +
                               " SBAS.");
    }
  }

  if (getRosBoolean("nmea/set") && !gps->configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware7::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav PVT (always does so since fix information is published
  // from this)
  gps->subscribe<ublox_msgs::NavPVT7>(std::bind(
        &UbloxFirmware7Plus::callbackNavPvt, this, std::placeholders::_1),
        1);

  // Subscribe to Nav SVINFO
  if (getRosBoolean("publish/nav/svinfo")) {
    gps->subscribe<ublox_msgs::NavSVINFO>([this](const ublox_msgs::NavSVINFO &m) { nav_svinfo_pub_.publish(m); },
                                          kNavSvInfoSubscribeRate);
  }

  // Subscribe to Mon HW
  if (getRosBoolean("publish/mon/hw")) {
    gps->subscribe<ublox_msgs::MonHW>([this](const ublox_msgs::MonHW &m) { mon_hw_pub_.publish(m); },
                                      1);
  }
}

//
// Ublox Version 8
//
void UbloxFirmware8::getRosParams() {
  // UPD SOS configuration
  clear_bbr_ = getRosBoolean("clear_bbr");
  save_on_shutdown_ = getRosBoolean("save_on_shutdown");

  // GNSS enable/disable
  enable_gps_ = getRosBoolean("gnss/gps");
  enable_galileo_ = getRosBoolean("gnss/galileo");
  enable_beidou_ = getRosBoolean("gnss/beidou");
  enable_imes_ = getRosBoolean("gnss/imes");
  enable_glonass_ = getRosBoolean("gnss/glonass");
  enable_qzss_ = getRosBoolean("gnss/qzss");

  // QZSS Signal Configuration
  getRosUint("gnss/qzss_sig_cfg", qzss_sig_cfg_,
              ublox_msgs::CfgGNSSBlock::SIG_CFG_QZSS_L1CA);

  if (enable_gps_ && !gnss_->isSupported("GPS")) {
    ROS_WARN("gnss/gps is true, but GPS GNSS is not supported by %s",
             "this device");
  }
  if (enable_glonass_ && !gnss_->isSupported("GLO")) {
    ROS_WARN("gnss/glonass is true, but GLONASS is not supported by %s",
             "this device");
  }
  if (enable_galileo_ && !gnss_->isSupported("GAL")) {
    ROS_WARN("gnss/galileo is true, but Galileo GNSS is not supported %s",
             "by this device");
  }
  if (enable_beidou_ && !gnss_->isSupported("BDS")) {
    ROS_WARN("gnss/beidou is true, but Beidou GNSS is not supported %s",
             "by this device");
  }
  if (enable_imes_ && !gnss_->isSupported("IMES")) {
    ROS_WARN("gnss/imes is true, but IMES GNSS is not supported by %s",
             "this device");
  }
  if (enable_qzss_ && !gnss_->isSupported("QZSS")) {
    ROS_WARN("gnss/qzss is true, but QZSS is not supported by this device");
  }
  if (getRosBoolean("gnss/sbas") && !gnss_->isSupported("SBAS")) {
    ROS_WARN("gnss/sbas is true, but SBAS is not supported by this device");
  }

  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::NavSatStatus::SERVICE_GPS
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GLONASS
      + (enable_beidou_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_COMPASS
      + (enable_galileo_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GALILEO;

  //
  // NMEA Configuration
  //
  if (getRosBoolean("nmea/set")) {
    bool compat, consider;
    cfg_nmea_.version = cfg_nmea_.VERSION; // message version

    // Verify that parameters are set
    if (!getRosUint("nmea/version", cfg_nmea_.nmea_version)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/version must be set");
    }
    if (!getRosUint("nmea/num_sv", cfg_nmea_.num_sv)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                "true, therefore nmea/num_sv must be set");
    }
    if (!getRosUint("nmea/sv_numbering", cfg_nmea_.sv_numbering)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/sv_numbering must be set");
    }
    if (!nh->getParam("nmea/compat", compat)) {
        throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/compat must be set");
    }
    if (!nh->getParam("nmea/consider", consider)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/consider must be set");
    }

    // set flags
    cfg_nmea_.flags = compat ? cfg_nmea_.FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= consider ? cfg_nmea_.FLAGS_CONSIDER : 0;
    cfg_nmea_.flags |= getRosBoolean("nmea/limit82") ? cfg_nmea_.FLAGS_LIMIT82 : 0;
    cfg_nmea_.flags |= getRosBoolean("nmea/high_prec") ? cfg_nmea_.FLAGS_HIGH_PREC : 0;
    // set filter
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/pos") ? cfg_nmea_.FILTER_POS : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/msk_pos") ? cfg_nmea_.FILTER_MSK_POS : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/time") ? cfg_nmea_.FILTER_TIME : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/date") ? cfg_nmea_.FILTER_DATE : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/gps_only") ? cfg_nmea_.FILTER_GPS_ONLY : 0;
    cfg_nmea_.filter |= getRosBoolean("nmea/filter/track") ? cfg_nmea_.FILTER_TRACK : 0;
    // set gnssToFilter
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/gps") ? cfg_nmea_.GNSS_TO_FILTER_GPS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/sbas") ? cfg_nmea_.GNSS_TO_FILTER_SBAS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/qzss") ? cfg_nmea_.GNSS_TO_FILTER_QZSS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/glonass") ? cfg_nmea_.GNSS_TO_FILTER_GLONASS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean("nmea/gnssToFilter/beidou") ? cfg_nmea_.GNSS_TO_FILTER_BEIDOU : 0;

    getRosUint("nmea/main_talker_id", cfg_nmea_.main_talker_id);
    getRosUint("nmea/gsv_talker_id", cfg_nmea_.gsv_talker_id);

    std::vector<uint8_t> bds_talker_id;
    getRosUint("nmea/bds_talker_id", bds_talker_id);
    cfg_nmea_.bds_talker_id[0] = bds_talker_id[0];
    cfg_nmea_.bds_talker_id[1] = bds_talker_id[1];
  }
}

bool UbloxFirmware8::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  if (clear_bbr_) {
    // clear flash memory
    if (!gps->clearBbr()) {
      ROS_ERROR("u-blox failed to clear flash memory");
    }
  }

  gps->setSaveOnShutdown(save_on_shutdown_);

  //
  // Configure the GNSS, only if the configuration is different
  //
  // First, get the current GNSS configuration
  ublox_msgs::CfgGNSS cfg_gnss;
  if (gps->poll(cfg_gnss)) {
    ROS_DEBUG("Read GNSS config.");
    ROS_DEBUG("Num. tracking channels in hardware: %i", cfg_gnss.num_trk_ch_hw);
    ROS_DEBUG("Num. tracking channels to use: %i", cfg_gnss.num_trk_ch_use);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  // Then, check the configuration for each GNSS. If it is different, change it.
  bool correct = true;
  for (int i = 0; i < cfg_gnss.blocks.size(); i++) {
    ublox_msgs::CfgGNSSBlock block = cfg_gnss.blocks[i];
    if (block.gnss_id == block.GNSS_ID_GPS
        && enable_gps_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_gps_;
      ROS_DEBUG("GPS Configuration is different");
    } else if (block.gnss_id == block.GNSS_ID_SBAS
               && getRosBoolean("gnss/sbas") != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | getRosBoolean("gnss/sbas");
      ROS_DEBUG("SBAS Configuration is different");
    } else if (block.gnss_id == block.GNSS_ID_GALILEO
               && enable_galileo_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_galileo_;
      ROS_DEBUG("Galileo GNSS Configuration is different");
    } else if (block.gnss_id == block.GNSS_ID_BEIDOU
               && enable_beidou_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_beidou_;
      ROS_DEBUG("BeiDou Configuration is different");
    } else if (block.gnss_id == block.GNSS_ID_IMES
               && enable_imes_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_imes_;
    } else if (block.gnss_id == block.GNSS_ID_QZSS
               && (enable_qzss_ != (block.flags & block.FLAGS_ENABLE)
               || (enable_qzss_
               && qzss_sig_cfg_ != (block.flags & block.FLAGS_SIG_CFG_MASK)))) {
      ROS_DEBUG("QZSS Configuration is different %u, %u",
                block.flags & block.FLAGS_ENABLE,
                enable_qzss_);
      correct = false;
      ROS_DEBUG("QZSS Configuration: %u", block.flags);
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_qzss_;
      ROS_DEBUG("QZSS Configuration: %u", cfg_gnss.blocks[i].flags);
      if (enable_qzss_) {
        // Only change sig cfg if enabling
        cfg_gnss.blocks[i].flags |= qzss_sig_cfg_;
      }
    } else if (block.gnss_id == block.GNSS_ID_GLONASS
               && enable_glonass_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_glonass_;
      ROS_DEBUG("GLONASS Configuration is different");
    }
  }

  // If the GNSS is already configured correctly, do not re-configure GNSS
  // since this requires a cold reset
  if (correct) {
    ROS_DEBUG("U-Blox GNSS configuration is correct. GNSS not re-configured.");
  } else if (!gps->configGnss(cfg_gnss, std::chrono::seconds(15))) {
    throw std::runtime_error(std::string("Failed to cold reset device ") +
                             "after configuring GNSS");
  }

  //
  // NMEA config
  //
  if (getRosBoolean("nmea/set") && !gps->configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware8::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav PVT
  gps->subscribe<ublox_msgs::NavPVT>(
    std::bind(&UbloxFirmware7Plus::callbackNavPvt, this, std::placeholders::_1), 1);

  // Subscribe to Nav SAT messages
  if (getRosBoolean("publish/nav/sat")) {
    gps->subscribe<ublox_msgs::NavSAT>([this](const ublox_msgs::NavSAT &m) { nav_sat_pub_.publish(m); },
                                       kNavSvInfoSubscribeRate);
  }

  // Subscribe to Mon HW
  if (getRosBoolean("publish/mon/hw")) {
    gps->subscribe<ublox_msgs::MonHW>([this](const ublox_msgs::MonHW &m) { mon_hw_pub_.publish(m); },
                                      1);
  }

  // Subscribe to RTCM messages
  if (getRosBoolean("publish/rxm/rtcm")) {
    gps->subscribe<ublox_msgs::RxmRTCM>([this](const ublox_msgs::RxmRTCM &m) { rxm_rtcm_pub_.publish(m); },
                                        1);
  }
}

UbloxFirmware9::UbloxFirmware9(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss)
  : UbloxFirmware8(frame_id, updater, freq_diag, gnss)
{
}

//
// Raw Data Products
//
RawDataProduct::RawDataProduct(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater)
  : nav_rate_(nav_rate), meas_rate_(meas_rate), updater_(updater) {
  rxm_raw_pub_ = nh->advertise<ublox_msgs::RxmRAW>("rxmraw", 1);
  rxm_sfrb_pub_ = nh->advertise<ublox_msgs::RxmSFRB>("rxmsfrb", 1);
  rxm_eph_pub_ = nh->advertise<ublox_msgs::RxmEPH>("rxmeph", 1);
  rxm_alm_pub_ = nh->advertise<ublox_msgs::RxmALM>("rxmalm", 1);
}

void RawDataProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to RXM Raw
  if (getRosBoolean("publish/rxm/raw")) {
    gps->subscribe<ublox_msgs::RxmRAW>([this](const ublox_msgs::RxmRAW &m) { rxm_raw_pub_.publish(m); },
                                       1);
  }

  // Subscribe to RXM SFRB
  if (getRosBoolean("publish/rxm/sfrb")) {
    gps->subscribe<ublox_msgs::RxmSFRB>([this](const ublox_msgs::RxmSFRB &m) { rxm_sfrb_pub_.publish(m); },
                                        1);
  }

  // Subscribe to RXM EPH
  if (getRosBoolean("publish/rxm/eph")) {
    gps->subscribe<ublox_msgs::RxmEPH>([this](const ublox_msgs::RxmEPH &m) { rxm_eph_pub_.publish(m); },
                                       1);
  }

  // Subscribe to RXM ALM
  if (getRosBoolean("publish/rxm/almRaw")) {
    gps->subscribe<ublox_msgs::RxmALM>([this](const ublox_msgs::RxmALM &m) { rxm_alm_pub_.publish(m); },
                                       1);
  }
}

void RawDataProduct::initializeRosDiagnostics() {
  if (getRosBoolean("publish/rxm/raw")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmraw", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
  if (getRosBoolean("publish/rxm/sfrb")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmsfrb", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
  if (getRosBoolean("publish/rxm/eph")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmeph", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
  if (getRosBoolean("publish/rxm/almRaw")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmalm", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
}

//
// u-blox ADR devices, partially implemented
//
AdrUdrProduct::AdrUdrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater)
  : nav_rate_(nav_rate), meas_rate_(meas_rate), frame_id_(frame_id), updater_(updater)
{
  imu_pub_ =
    nh->advertise<sensor_msgs::Imu>("imu_meas", 1);
  time_ref_pub_ =
    nh->advertise<sensor_msgs::TimeReference>("interrupt_time", 1);
  nav_att_pub_ = nh->advertise<ublox_msgs::NavATT>("navatt", 1);
  esf_ins_pub_ = nh->advertise<ublox_msgs::EsfINS>("esfins", 1);
  esf_meas_pub_ = nh->advertise<ublox_msgs::EsfMEAS>("esfmeas", 1);
  esf_raw_pub_ = nh->advertise<ublox_msgs::EsfRAW>("esfraw", 1);
  esf_status_pub_ = nh->advertise<ublox_msgs::EsfSTATUS>("esfstatus", 1);
  hnr_pvt_pub_ = nh->advertise<ublox_msgs::HnrPVT>("hnrpvt", 1);
}

void AdrUdrProduct::getRosParams() {
  nh->param("use_adr", use_adr_, true);
  // Check the nav rate
  float nav_rate_hz = 1000 / (meas_rate_ * nav_rate_);
  if (nav_rate_hz != 1) {
    ROS_WARN("Nav Rate recommended to be 1 Hz");
  }
}

bool AdrUdrProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  if (!gps->setUseAdr(use_adr_)) {
    throw std::runtime_error(std::string("Failed to ")
                             + (use_adr_ ? "enable" : "disable") + "use_adr");
  }
  return true;
}

void AdrUdrProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to NAV ATT messages
  if (getRosBoolean("publish/nav/att")) {
    gps->subscribe<ublox_msgs::NavATT>([this](const ublox_msgs::NavATT &m) { nav_att_pub_.publish(m); },
                                       1);
  }

  // Subscribe to ESF INS messages
  if (getRosBoolean("publish/esf/ins")) {
    gps->subscribe<ublox_msgs::EsfINS>([this](const ublox_msgs::EsfINS &m) { esf_ins_pub_.publish(m); },
                                       1);
  }

  // Subscribe to ESF Meas messages
  if (getRosBoolean("publish/esf/meas")) {
    gps->subscribe<ublox_msgs::EsfMEAS>([this](const ublox_msgs::EsfMEAS &m) { esf_meas_pub_.publish(m); },
                                        1);
    // also publish sensor_msgs::Imu
    gps->subscribe<ublox_msgs::EsfMEAS>(std::bind(
      &AdrUdrProduct::callbackEsfMEAS, this, std::placeholders::_1), 1);
  }

  // Subscribe to ESF Raw messages
  if (getRosBoolean("publish/esf/raw")) {
    gps->subscribe<ublox_msgs::EsfRAW>([this](const ublox_msgs::EsfRAW &m) { esf_raw_pub_.publish(m); },
                                       1);
  }

  // Subscribe to ESF Status messages
  if (getRosBoolean("publish/esf/status")) {
    gps->subscribe<ublox_msgs::EsfSTATUS>([this](const ublox_msgs::EsfSTATUS &m) { esf_status_pub_.publish(m); },
                                          1);
  }

  // Subscribe to HNR PVT messages
  if (getRosBoolean("publish/hnr/pvt")) {
    gps->subscribe<ublox_msgs::HnrPVT>([this](const ublox_msgs::HnrPVT &m) { hnr_pvt_pub_.publish(m); },
                                       1);
  }
}

void AdrUdrProduct::callbackEsfMEAS(const ublox_msgs::EsfMEAS &m) {
  if (getRosBoolean("publish/esf/meas")) {
    imu_.header.stamp = ros::Time::now();
    imu_.header.frame_id = frame_id_;

    float deg_per_sec = pow(2, -12);
    float m_per_sec_sq = pow(2, -10);
    float deg_c = 1e-2;

    std::vector<unsigned int> imu_data = m.data;
    for (int i=0; i < imu_data.size(); i++){
      unsigned int data_type = imu_data[i] >> 24; //grab the last six bits of data
      double data_sign = (imu_data[i] & (1 << 23)); //grab the sign (+/-) of the rest of the data
      unsigned int data_value = imu_data[i] & 0x7FFFFF; //grab the rest of the data...should be 23 bits

      if (data_sign == 0) {
        data_sign = -1;
      } else {
        data_sign = 1;
      }

      //ROS_INFO("data sign (+/-): %f", data_sign); //either 1 or -1....set by bit 23 in the data bitarray

      imu_.orientation_covariance[0] = -1;
      imu_.linear_acceleration_covariance[0] = -1;
      imu_.angular_velocity_covariance[0] = -1;

      if (data_type == 14) {
        if (data_sign == 1) {
	  imu_.angular_velocity.x = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.x = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 16) {
        //ROS_INFO("data_sign: %f", data_sign);
        //ROS_INFO("data_value: %u", data_value * m);
        if (data_sign == 1) {
	  imu_.linear_acceleration.x = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.x = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 13) {
        if (data_sign == 1) {
	  imu_.angular_velocity.y = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.y = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 17) {
        if (data_sign == 1) {
	  imu_.linear_acceleration.y = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.y = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 5) {
        if (data_sign == 1) {
	  imu_.angular_velocity.z = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.z = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 18) {
        if (data_sign == 1) {
	  imu_.linear_acceleration.z = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.z = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 12) {
        //ROS_INFO("Temperature in celsius: %f", data_value * deg_c);
      } else {
        ROS_INFO("data_type: %u", data_type);
        ROS_INFO("data_value: %u", data_value);
      }

      // create time ref message and put in the data
      //t_ref_.header.seq = m.risingEdgeCount;
      //t_ref_.header.stamp = ros::Time::now();
      //t_ref_.header.frame_id = frame_id_;

      //t_ref_.time_ref = ros::Time((m.wnR * 604800 + m.towMsR / 1000), (m.towMsR % 1000) * 1000000 + m.towSubMsR);

      //std::ostringstream src;
      //src << "TIM" << int(m.ch);
      //t_ref_.source = src.str();

      t_ref_.header.stamp = ros::Time::now(); // create a new timestamp
      t_ref_.header.frame_id = frame_id_;

      time_ref_pub_.publish(t_ref_);
      imu_pub_.publish(imu_);
    }
  }

  updater_->force_update();
}
//
// u-blox High Precision GNSS Reference Station
//

HpgRefProduct::HpgRefProduct(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms)
  : nav_rate_(nav_rate), meas_rate_(meas_rate), updater_(updater), rtcms_(rtcms)
{
  navsvin_pub_ =
    nh->advertise<ublox_msgs::NavSVIN>("navsvin", 1);
}

void HpgRefProduct::getRosParams() {
  if (getRosBoolean("config_on_startup")) {
    if (nav_rate_ * meas_rate_ != 1000) {
      ROS_WARN("For HPG Ref devices, nav_rate should be exactly 1 Hz.");
    }

    if (!getRosUint("tmode3", tmode3_)) {
      throw std::runtime_error("Invalid settings: TMODE3 must be set");
    }

    if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_FIXED) {
      if (!nh->getParam("arp/position", arp_position_)) {
        throw std::runtime_error(std::string("Invalid settings: arp/position ")
                                + "must be set if TMODE3 is fixed");
      }
      if (!getRosInt("arp/position_hp", arp_position_hp_)) {
        throw std::runtime_error(std::string("Invalid settings: arp/position_hp ")
                                + "must be set if TMODE3 is fixed");
      }
      if (!nh->getParam("arp/acc", fixed_pos_acc_)) {
        throw std::runtime_error(std::string("Invalid settings: arp/acc ")
                                + "must be set if TMODE3 is fixed");
      }
      if (!nh->getParam("arp/lla_flag", lla_flag_)) {
        ROS_WARN("arp/lla_flag param not set, assuming ARP coordinates are %s",
                "in ECEF");
        lla_flag_ = false;
      }
    } else if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
      nh->param("sv_in/reset", svin_reset_, true);
      if (!getRosUint("sv_in/min_dur", sv_in_min_dur_)) {
        throw std::runtime_error(std::string("Invalid settings: sv_in/min_dur ")
                                + "must be set if TMODE3 is survey-in");
      }
      if (!nh->getParam("sv_in/acc_lim", sv_in_acc_lim_)) {
        throw std::runtime_error(std::string("Invalid settings: sv_in/acc_lim ")
                                + "must be set if TMODE3 is survey-in");
      }
    } else if (tmode3_ != ublox_msgs::CfgTMODE3::FLAGS_MODE_DISABLED) {
      throw std::runtime_error(std::string("tmode3 param invalid. See CfgTMODE3")
                              + " flag constants for possible values.");
    }
  }
}

bool HpgRefProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  // Configure TMODE3
  if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_DISABLED) {
    if (!gps->disableTmode3()) {
      throw std::runtime_error("Failed to disable TMODE3.");
    }
    mode_ = DISABLED;
  } else if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_FIXED) {
    if (!gps->configTmode3Fixed(lla_flag_, arp_position_, arp_position_hp_,
                               fixed_pos_acc_)) {
      throw std::runtime_error("Failed to set TMODE3 to fixed.");
    }
    if (!gps->configRtcm(rtcms_)) {
      throw std::runtime_error("Failed to set RTCM rates");
    }
    mode_ = FIXED;
  } else if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
    if (!svin_reset_) {
      ublox_msgs::NavSVIN nav_svin;
      if (!gps->poll(nav_svin)) {
        throw std::runtime_error(std::string("Failed to poll NavSVIN while") +
                                 " configuring survey-in");
      }
      // Don't reset survey-in if it's already active
      if (nav_svin.active) {
        mode_ = SURVEY_IN;
        return true;
      }
      // Don't reset survey-in if it already has a valid value
      if (nav_svin.valid) {
        setTimeMode(gps);
        return true;
      }
      ublox_msgs::NavPVT nav_pvt;
      if (!gps->poll(nav_pvt)) {
        throw std::runtime_error(std::string("Failed to poll NavPVT while") +
                                 " configuring survey-in");
      }
      // Don't reset survey in if in time mode with a good fix
      if (nav_pvt.fix_type == nav_pvt.FIX_TYPE_TIME_ONLY
          && nav_pvt.flags & nav_pvt.FLAGS_GNSS_FIX_OK) {
        setTimeMode(gps);
        return true;
      }
    }
    // Reset the Survey In
    // For Survey in, meas rate must be at least 1 Hz
    uint16_t meas_rate_temp = meas_rate_ < 1000 ? meas_rate_ : 1000; // [ms]
    // If measurement period isn't a factor of 1000, set to default
    if (1000 % meas_rate_temp != 0) {
      meas_rate_temp = kDefaultMeasPeriod;
    }
    // Set nav rate to 1 Hz during survey in
    if (!gps->configRate(meas_rate_temp, (int) 1000 / meas_rate_temp)) {
      throw std::runtime_error(std::string("Failed to set nav rate to 1 Hz") +
                               "before setting TMODE3 to survey-in.");
    }
    // As recommended in the documentation, first disable, then set to survey in
    if (!gps->disableTmode3()) {
      ROS_ERROR("Failed to disable TMODE3 before setting to survey-in.");
    } else {
      mode_ = DISABLED;
    }
    // Set to Survey in mode
    if (!gps->configTmode3SurveyIn(sv_in_min_dur_, sv_in_acc_lim_)) {
      throw std::runtime_error("Failed to set TMODE3 to survey-in.");
    }
    mode_ = SURVEY_IN;
  }
  return true;
}

void HpgRefProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav Survey-In
  // Save off the gps pointer so we can use it in the callback later.
  gps_ = gps;
  gps->subscribe<ublox_msgs::NavSVIN>(std::bind(
      &HpgRefProduct::callbackNavSvIn, this, std::placeholders::_1), 1);
}

void HpgRefProduct::callbackNavSvIn(const ublox_msgs::NavSVIN& m) {
  if (getRosBoolean("publish/nav/svin")) {
    navsvin_pub_.publish(m);
  }

  last_nav_svin_ = m;

  if (!m.active && m.valid && mode_ == SURVEY_IN) {
    setTimeMode(gps_);
  }

  updater_->update();
}

bool HpgRefProduct::setTimeMode(std::shared_ptr<ublox_gps::Gps> gps) {
  ROS_INFO("Setting mode (internal state) to Time Mode");
  mode_ = TIME;

  // Set the Measurement & nav rate to user config
  // (survey-in sets nav_rate to 1 Hz regardless of user setting)
  if (!gps->configRate(meas_rate_, nav_rate_)) {
    ROS_ERROR("Failed to set measurement rate to %d ms %s %d", meas_rate_,
              "navigation rate to ", nav_rate_);
  }
  // Enable the RTCM out messages
  if (!gps->configRtcm(rtcms_)) {
    ROS_ERROR("Failed to configure RTCM IDs");
    return false;
  }
  return true;
}

void HpgRefProduct::initializeRosDiagnostics() {
  updater_->add("TMODE3", this, &HpgRefProduct::tmode3Diagnostics);
  updater_->force_update();
}

void HpgRefProduct::tmode3Diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (mode_ == INIT) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Not configured";
  } else if (mode_ == DISABLED){
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "Disabled";
  } else if (mode_ == SURVEY_IN) {
    if (!last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.message = "Survey-In inactive and invalid";
    } else if (last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.message = "Survey-In active but invalid";
    } else if (!last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Survey-In complete";
    } else if (last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Survey-In active and valid";
    }

    stat.add("iTOW [ms]", last_nav_svin_.i_tow);
    stat.add("Duration [s]", last_nav_svin_.dur);
    stat.add("# observations", last_nav_svin_.obs);
    stat.add("Mean X [m]", last_nav_svin_.mean_x * 1e-2);
    stat.add("Mean Y [m]", last_nav_svin_.mean_y * 1e-2);
    stat.add("Mean Z [m]", last_nav_svin_.mean_z * 1e-2);
    stat.add("Mean X HP [m]", last_nav_svin_.mean_xhp * 1e-4);
    stat.add("Mean Y HP [m]", last_nav_svin_.mean_yhp * 1e-4);
    stat.add("Mean Z HP [m]", last_nav_svin_.mean_zhp * 1e-4);
    stat.add("Mean Accuracy [m]", last_nav_svin_.mean_acc * 1e-4);
  } else if (mode_ == FIXED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "Fixed Position";
  } else if (mode_ == TIME) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "Time";
  }
}

//
// U-Blox High Precision GNSS Rover
//
HpgRovProduct::HpgRovProduct(uint16_t nav_rate, std::shared_ptr<diagnostic_updater::Updater> updater)
  : nav_rate_(nav_rate), updater_(updater)
{
  nav_rel_pos_ned_pub_ =
    nh->advertise<ublox_msgs::NavRELPOSNED>("navrelposned", 1);
}

void HpgRovProduct::getRosParams() {
  // default to float, see CfgDGNSS message for details
  getRosUint("dgnss_mode", dgnss_mode_,
              ublox_msgs::CfgDGNSS::DGNSS_MODE_RTK_FIXED);
}

bool HpgRovProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  // Configure the DGNSS
  if (!gps->setDgnss(dgnss_mode_)) {
    throw std::runtime_error(std::string("Failed to Configure DGNSS"));
  }
  return true;
}

void HpgRovProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  gps->subscribe<ublox_msgs::NavRELPOSNED>(std::bind(
     &HpgRovProduct::callbackNavRelPosNed, this, std::placeholders::_1), 1);
}

void HpgRovProduct::initializeRosDiagnostics() {
  freq_rtcm_ = UbloxTopicDiagnostic(std::string("rxmrtcm"),
                                    kRtcmFreqMin, kRtcmFreqMax,
                                    kRtcmFreqTol, kRtcmFreqWindow, updater_);
  updater_->add("Carrier Phase Solution", this,
                &HpgRovProduct::carrierPhaseDiagnostics);
  updater_->force_update();
}

void HpgRovProduct::carrierPhaseDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  uint32_t carr_soln = last_rel_pos_.flags & last_rel_pos_.FLAGS_CARR_SOLN_MASK;
  stat.add("iTow", last_rel_pos_.i_tow);
  if (carr_soln & last_rel_pos_.FLAGS_CARR_SOLN_NONE ||
      !(last_rel_pos_.flags & last_rel_pos_.FLAGS_DIFF_SOLN &&
        last_rel_pos_.flags & last_rel_pos_.FLAGS_REL_POS_VALID)) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "None";
  } else {
    if (carr_soln & last_rel_pos_.FLAGS_CARR_SOLN_FLOAT) {
      stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.message = "Float";
    } else if (carr_soln & last_rel_pos_.FLAGS_CARR_SOLN_FIXED) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Fixed";
    }
    stat.add("Ref Station ID", last_rel_pos_.ref_station_id);

    double rel_pos_n = (last_rel_pos_.rel_pos_n
                       + (last_rel_pos_.rel_pos_hpn * 1e-2)) * 1e-2;
    double rel_pos_e = (last_rel_pos_.rel_pos_e
                       + (last_rel_pos_.rel_pos_hpe * 1e-2)) * 1e-2;
    double rel_pos_d = (last_rel_pos_.rel_pos_d
                       + (last_rel_pos_.rel_pos_hpd * 1e-2)) * 1e-2;
    stat.add("Relative Position N [m]", rel_pos_n);
    stat.add("Relative Accuracy N [m]", last_rel_pos_.acc_n * 1e-4);
    stat.add("Relative Position E [m]", rel_pos_e);
    stat.add("Relative Accuracy E [m]", last_rel_pos_.acc_e * 1e-4);
    stat.add("Relative Position D [m]", rel_pos_d);
    stat.add("Relative Accuracy D [m]", last_rel_pos_.acc_d * 1e-4);
  }
}

void HpgRovProduct::callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED &m) {
  if (getRosBoolean("publish/nav/relposned")) {
    nav_rel_pos_ned_pub_.publish(m);
  }

  last_rel_pos_ = m;
  updater_->update();
}

//
// U-Blox High Precision Positioning Receiver
//
HpPosRecProduct::HpPosRecProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms)
  : HpgRefProduct(nav_rate, meas_rate, updater, rtcms), frame_id_(frame_id)
{
  nav_relposned_pub_ =
    nh->advertise<ublox_msgs::NavRELPOSNED9>("navrelposned", 1);

  imu_pub_ =
    nh->advertise<sensor_msgs::Imu>("navheading", 1);
}

void HpPosRecProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Whether to publish Nav Relative Position NED
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  gps->subscribe<ublox_msgs::NavRELPOSNED9>(std::bind(
     &HpPosRecProduct::callbackNavRelPosNed, this, std::placeholders::_1), 1);
}

void HpPosRecProduct::callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED9 &m) {
  if (getRosBoolean("publish/nav/relposned")) {
    nav_relposned_pub_.publish(m);
  }

  if (getRosBoolean("publish/nav/heading")) {
    imu_.header.stamp = ros::Time::now();
    imu_.header.frame_id = frame_id_;

    imu_.linear_acceleration_covariance[0] = -1;
    imu_.angular_velocity_covariance[0] = -1;

    double heading = static_cast<double>(m.rel_pos_heading) * 1e-5 / 180.0 * M_PI;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, heading);
    imu_.orientation.x = orientation[0];
    imu_.orientation.y = orientation[1];
    imu_.orientation.z = orientation[2];
    imu_.orientation.w = orientation[3];
    // Only heading is reported with an accuracy in 0.1mm units
    imu_.orientation_covariance[0] = 1000.0;
    imu_.orientation_covariance[4] = 1000.0;
    imu_.orientation_covariance[8] = pow(m.acc_heading / 10000.0, 2);

    imu_pub_.publish(imu_);
  }

  last_rel_pos_ = m;
  updater_->update();
}

//
// U-Blox Time Sync Products, partially implemented.
//
TimProduct::TimProduct(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater) : frame_id_(frame_id), updater_(updater)
{
  timtm2_pub_ =
    nh->advertise<ublox_msgs::TimTM2>("timtm2", 1);
  interrupt_time_pub_ =
    nh->advertise<sensor_msgs::TimeReference>("interrupt_time", 1);
  rxm_sfrb_pub_ = nh->advertise<ublox_msgs::RxmSFRBX>("rxmsfrb", 1);
  rxm_raw_pub_ = nh->advertise<ublox_msgs::RxmRAWX>("rxmraw", 1);
}

void TimProduct::getRosParams() {
}

bool TimProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  uint8_t r = 1;
  // Configure the reciever
  if (!gps->setUTCtime()) {
    throw std::runtime_error(std::string("Failed to Configure TIM Product to UTC Time"));
  }

  if (!gps->setTimtm2(r)) {
    throw std::runtime_error(std::string("Failed to Configure TIM Product"));
  }

  return true;
}

void TimProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  gps->subscribe<ublox_msgs::TimTM2>(std::bind(
    &TimProduct::callbackTimTM2, this, std::placeholders::_1), 1);

  ROS_INFO("Subscribed to TIM-TM2 messages on topic tim/tm2");

  // Subscribe to SFRBX messages
  if (getRosBoolean("publish/rxm/sfrb")) {
    gps->subscribe<ublox_msgs::RxmSFRBX>([this](const ublox_msgs::RxmSFRBX &m) { rxm_sfrb_pub_.publish(m); },
                                         1);
  }

   // Subscribe to RawX messages
   if (getRosBoolean("publish/rxm/raw")) {
     gps->subscribe<ublox_msgs::RxmRAWX>([this](const ublox_msgs::RxmRAWX &m) { rxm_raw_pub_.publish(m); },
                                         1);
   }
}

void TimProduct::callbackTimTM2(const ublox_msgs::TimTM2 &m) {

  if (getRosBoolean("publish/tim/tm2")) {
    // create time ref message and put in the data
    t_ref_.header.seq = m.rising_edge_count;
    t_ref_.header.stamp = ros::Time::now();
    t_ref_.header.frame_id = frame_id_;

    t_ref_.time_ref = ros::Time((m.wn_r * 604800 + m.tow_ms_r / 1000), (m.tow_ms_r % 1000) * 1000000 + m.tow_sub_ms_r);

    std::ostringstream src;
    src << "TIM" << int(m.ch);
    t_ref_.source = src.str();

    t_ref_.header.stamp = ros::Time::now(); // create a new timestamp
    t_ref_.header.frame_id = frame_id_;

    timtm2_pub_.publish(m);
    interrupt_time_pub_.publish(t_ref_);
  }

  updater_->force_update();
}

void TimProduct::initializeRosDiagnostics() {
  updater_->force_update();
}

}  // namespace ublox_node

int main(int argc, char** argv) {
  ros::init(argc, argv, "ublox_gps");
  ublox_node::nh = std::make_shared<ros::NodeHandle>("~");
  int debug;
  ublox_node::nh->param("debug", debug, 1);
  if (debug) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  ublox_node::UbloxNode node;
  return 0;
}
