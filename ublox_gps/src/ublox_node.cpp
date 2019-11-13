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

#include <algorithm>
#include <cmath>
#include <memory>
#include <regex>
#include <string>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <ublox_msgs/msg/aid_alm.hpp>
#include <ublox_msgs/msg/aid_eph.hpp>
#include <ublox_msgs/msg/aid_hui.hpp>
#include <ublox_msgs/msg/cfg_inf.hpp>
#include <ublox_msgs/msg/cfg_inf_block.hpp>
#include <ublox_msgs/msg/cfg_nav5.hpp>
#include <ublox_msgs/msg/cfg_prt.hpp>
#include <ublox_msgs/msg/inf.hpp>
#include <ublox_msgs/msg/mon_ver.hpp>
#include <ublox_msgs/msg/nav_clock.hpp>
#include <ublox_msgs/msg/nav_posecef.hpp>
#include <ublox_msgs/msg/nav_status.hpp>
#include <ublox_msgs/ublox_msgs.hpp>

#include <ublox_gps/ublox_node.hpp>

namespace ublox_gps {

/**
 * @brief Check that the parameter is above the minimum.
 * @param val the value to check
 * @param min the minimum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error if it is below the minimum
 */
template <typename V, typename T>
void checkMin(V val, T min, std::string name) {
  if (val < min) {
    std::stringstream oss;
    oss << "Invalid settings: " << name << " must be > " << min;
    throw std::runtime_error(oss.str());
  }
}

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
  std::transform(lower.begin(), lower.end(), lower.begin(), std::tolower);
  if (lower == "portable") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_PORTABLE;
  } else if (lower == "stationary") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_STATIONARY;
  } else if (lower == "pedestrian") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_PEDESTRIAN;
  } else if (lower == "automotive") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AUTOMOTIVE;
  } else if (lower == "sea") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_SEA;
  } else if (lower == "airborne1") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_1G;
  } else if (lower == "airborne2") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_2G;
  } else if (lower == "airborne4") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_4G;
  } else if (lower == "wristwatch") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_WRIST_WATCH;
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
  std::transform(lower.begin(), lower.end(), lower.begin(), std::tolower);
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
UbloxNode::UbloxNode(const rclcpp::NodeOptions & options) : rclcpp::Node("ublox_node", options) {
  updater_ = std::make_unique<diagnostic_updater::Updater>();
  updater_->setHardwareID("ublox");

  updater_->add("fix", this, &UbloxFirmware::fixDiagnostic);
  updater_->force_update();

  initialize();
}

void UbloxNode::addFirmwareInterface() {
  int ublox_version;
  if (protocol_version_ < 14) {
    components_.push_back(std::make_unique<UbloxFirmware6>());
    ublox_version = 6;
  } else if (protocol_version_ >= 14 && protocol_version_ <= 15) {
    components_.push_back(std::make_unique<UbloxFirmware7>());
    ublox_version = 7;
  } else if (protocol_version_ > 15 && protocol_version_ <= 23) {
    components_.push_back(std::make_unique<UbloxFirmware8>());
    ublox_version = 8;
  } else {
    components_.push_back(std::make_unique<UbloxFirmware9>());
    ublox_version = 9;
  }

  ROS_INFO("U-Blox Firmware Version: %d", ublox_version);
}


void UbloxNode::addProductInterface(const std::string & product_category,
                                    const std::string & ref_rov) {
  if (product_category.compare("HPG") == 0 && ref_rov.compare("REF") == 0) {
    components_.push_back(std::make_unique<HpgRefProduct>());
  } else if (product_category.compare("HPG") == 0 && ref_rov.compare("ROV") == 0) {
    components_.push_back(std::make_unique<HpgRovProduct>());
  } else if (product_category.compare("HPG") == 0) {
    components_.push_back(std::make_unique<HpPosRecProduct>());
  } else if (product_category.compare("TIM") == 0) {
    components_.push_back(std::make_unique<TimProduct>());
  } else if (product_category.compare("ADR") == 0 ||
           product_category.compare("UDR") == 0) {
    components_.push_back(std::make_unique<AdrUdrProduct>());
  } else if (product_category.compare("FTS") == 0) {
    components_.push_back(std::make_unique<FtsProduct>());
  } else if (product_category.compare("SPG") != 0) {
    RCLCPP_WARN(get_logger(), "Product category %s %s from MonVER message not recognized %s",
             product_category.c_str(), ref_rov.c_str(),
             "options are HPG REF, HPG ROV, HPG #.#, TIM, ADR, UDR, FTS, SPG");
  }
}

void UbloxNode::getRosParams() {
  device_ = this->declare_parameter("device", std::string("/dev/ttyACM0"));
  frame_id_ = this->declare_parameter("frame_id", std::string("gps"));

  // Save configuration parameters
  load_.load_mask = this->declare_parameter("load.mask", 0);
  load_.device_mask = this->declare_parameter("load.device", 0);
  save_.save_mask = this->declare_parameter("save.mask", 0);
  save_.device_mask = this->declare_parameter("save.device", 0);

  // UART 1 params
  baudrate_ = this->declare_parameter("uart1.baudrate", 9600);
  // TODO(clalancette): this should be restricted to uint16
  uart_in_ = this->declare_parameter("uart1.in", ublox_msgs::msg::CfgPRT::PROTO_UBX
                                    | ublox_msgs::msg::CfgPRT::PROTO_NMEA
                                    | ublox_msgs::msg::CfgPRT::PROTO_RTCM);
  // TODO(clalancette): this should be restricted to uint16
  uart_out_ = this->declare_parameter("uart1.out", ublox_msgs::msg::CfgPRT::PROTO_UBX);
  // USB params
  set_usb_ = false;
  this->declare_parameter("usb.in");
  this->declare_parameter("usb.out");
  if (this->has_parameter("usb.in") || this->has_parameter("usb.out")) {
    set_usb_ = true;
    rclcpp::Parameter usbin = this->get_parameter("usb.in");
    if (usbin.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      throw std::runtime_error(std::string("usb/out is set, therefore ") +
        "usb/in must be set");
    }
    // TODO(clalancette): this should be restricted to uint16_t values
    usb_in_ = usbin.get_value<uint16_t>();

    rclcpp::Parameter usbout = this->get_parameter("usb.out");
    if (usbout.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      throw std::runtime_error(std::string("usb/in is set, therefore ") +
        "usb/out must be set");
    }
    // TODO(clalancette): this should be restricted to uint16_t values
    usb_out_ = usbout.get_value<uint16_t>();

    // TODO(clalancette): this should be restricted to uint16_t values
    usb_tx_ = this->declare_parameter("usb.tx_ready", 0);
  }
  // Measurement rate params
  rate_ = this->declare_parameter("rate", 4.0);
  // TODO(clalancette): this should be restricted to uint16
  nav_rate_ = this->declare_parameter("nav_rate", 1);  // # of measurement rate cycles
  // RTCM params
  // TODO(clalancette): Figure out vectors in ROS 2 parameters
  //getRosUint("rtcm/ids", rtcm_ids);  // RTCM output message IDs
  //getRosUint("rtcm/rates", rtcm_rates);  // RTCM output message rates
  // PPP: Advanced Setting
  enable_ppp_ = this->declare_parameter("enable_ppp", false);
  // SBAS params, only for some devices
  enable_sbas_ = this->declare_parameter("enable_sbas", false);
  // TODO(clalancette): this should be restricted to a uint8
  max_sbas_ = this->declare_parameter("sbas.max", 0);  // Maximum number of SBAS channels
  // TODO(clalancette): this should be restricted to a uint8
  sbas_usage_ = this->declare_parameter("sbas.usage", 0);
  dynamic_model_ = this->declare_parameter("dynamic_model", std::string("portable"));
  fix_mode_ = this->declare_parameter("fix_mode", std::string("auto"));
  dr_limit_ = this->declare_parameter("dr_limit", 0);  // Dead reckoning limit

  if (enable_ppp_) {
    RCLCPP_WARN(get_logger(), "Warning: PPP is enabled - this is an expert setting.");
  }

  checkMin(rate_, 0, "rate");

  if (rtcm_ids.size() != rtcm_rates.size()) {
    throw std::runtime_error(std::string("Invalid settings: size of rtcm_ids") +
                             " must match size of rtcm_rates");
  }

  dmodel_ = modelFromString(dynamic_model_);
  fmode_ = fixModeFromString(fix_mode_);

  set_dat_ = this->declare_parameter("dat.set", false);
  if (set_dat_) {
    std::vector<float> shift, rot;
    this->declare_parameter("dat.maj_a");
    this->declare_parameter("dat.flat");
    this->declare_parameter("dat.shift");
    this->declare_parameter("dat.rot");
    this->declare_parameter("dat.scale");

    if (!this->has_parameter("dat.maj_a")
        || !this->has_parameter("dat.flat")
        || !this->has_parameter("dat.shift")
        || !this->has_parameter("dat.rot")
        || !this->has_parameter("dat.scale")
      throw std::runtime_error(std::string("dat/set is true, therefore ") +
         "dat/majA, dat/flat, dat/shift, dat/rot, & dat/scale must be set");
    }
    cfg_data_.maj_a = this->get_parameter("dat.maj_a");
    cfg_data_.flat = this->get_parameter("dat.flat");
    shift = this->get_parameter("dat.shift");
    rot = this->get_parameter("dat.rot");
    cfg_data_.scale = this->get_parameter("dat.scale");

    if (shift.size() != 3 || rot.size() != 3) {
      throw std::runtime_error(std::string("size of dat/shift & dat/rot ") +
                               "must be 3");
    }
    checkRange(cfg_dat_.maj_a, 6300000.0, 6500000.0, "dat/maj_a");
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
  meas_rate = 1000 / rate_;

  // activate/deactivate any config
  config_on_startup_flag_ = this->declare_parameter("config_on_startup", true);

  // raw data stream logging
  rawDataStreamPa_.getRosParams();

  all_enable_ = this->declare_parameter("publish.all", false);
  inf_enable_ = this->declare_parameter("inf.all", true);
  nav_enable_ = this->declare_parameter("publish.nav.all", all_enable_);
  rxm_enable_ = this->declare_parameter("publish.rxm.all", all_enable_);
  aid_enable_ = this->declare_parameter("publish.aid.all", all_enable_);
  mon_enable_ = this->declare_parameter("publish.mon.all", all_enable_);

  nav_status_enable_ = this->declare_parameter("publish.nav.status", nav_enable_);
  nav_posecef_enable_ = this->declare_parameter("publish.nav.posecef", nav_enable_);
  nav_clock_enable_ = this->declare_parameter("publish.nav.clock", nav_enable_);

  inf_debug_enable_ = this->declare_parameter("inf.debug", false);
  inf_error_enable_ = this->declare_parameter("inf.error", inf_enable_);
  inf_notice_enable_ = this->declare_parameter("inf.notice", inf_enable_);
  inf_test_enable_ = this->declare_parameter("inf.test", inf_enable_);
  inf_warning_enable_ = this->declare_parameter("inf.warning", inf_enable_);

  aid_alm_enable_ = this->declare_parameter("publish.aid.alm", aid_enable_);
  aid_eph_enable_ = this->declare_parameter("publish.aid.eph", aid_enable_);
  aid_hui_enable_ = this->declare_parameter("publish.aid.hui", aid_enable_);

  diagnostic_period_ = this->declare_parameter("diagnostic_period", kDiagnosticPeriod);

  raw_data_enable_ = this->declare_parameter("raw_data", false);
}

void UbloxNode::pollMessages(const ros::TimerEvent& event) {
  static std::vector<uint8_t> payload(1, 1);
  if (enabled["aid_alm"]) {
    gps_.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM, payload);
  }
  if (enabled["aid_eph"]) {
    gps_.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH, payload);
  }
  if (enabled["aid_hui"]) {
    gps_.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI);
  }

  payload[0]++;
  if (payload[0] > 32) {
    payload[0] = 1;
  }
}

void UbloxNode::printInf(const ublox_msgs::msg::Inf &m, uint8_t id) {
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
  RCLCPP_DEBUG(get_logger(), "Subscribing to U-Blox messages");
  // subscribe messages

  // Nav Messages
  if (nav_status_enable_) {
    gps_.subscribe<ublox_msgs::msg::NavSTATUS>(std::bind(
        publish<ublox_msgs::msg::NavSTATUS>, std::placeholders::_1, "navstatus"), 1);
  }

  if (nav_posecef_enable_) {
    gps_.subscribe<ublox_msgs::msg::NavPOSECEF>(std::bind(
        publish<ublox_msgs::msg::NavPOSECEF>, std::placeholders::_1, "navposecef"), 1);
  }

  if (nav_clock_enable_) {
    gps_.subscribe<ublox_msgs::msg::NavCLOCK>(std::bind(
        publish<ublox_msgs::msg::NavCLOCK>, std::placeholders::_1, "navclock"), 1);
  }

  // INF messages
  if (inf_debug_enable_) {
    gps_.subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::DEBUG),
        ublox_msgs::Message::INF::DEBUG);
  }

  if (inf_error_enable_) {
    gps_.subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::ERROR),
        ublox_msgs::Message::INF::ERROR);
  }

  if (inf_notice_enable_) {
    gps_.subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::NOTICE),
        ublox_msgs::Message::INF::NOTICE);
  }

  if (inf_test_enable_) {
    gps_.subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::TEST),
        ublox_msgs::Message::INF::TEST);
  }

  if (inf_warning_enable_) {
    gps_.subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::WARNING),
        ublox_msgs::Message::INF::WARNING);
  }

  // AID messages
  if (aid_alm_enable_) {
    gps_.subscribe<ublox_msgs::msg::AidALM>(std::bind(
        publish<ublox_msgs::msg::AidALM>, std::placeholders::_1, "aidalm"), 1);
  }

  if (aid_eph_enable_) {
    gps_.subscribe<ublox_msgs::msg::AidEPH>(std::bind(
        publish<ublox_msgs::msg::AidEPH>, std::placeholders::_1, "aideph"), 1);
  }

  if (aid_hui_enable_) {
    gps_.subscribe<ublox_msgs::msg::AidHUI>(std::bind(
        publish<ublox_msgs::msg::AidHUI>, std::placeholders::_1, "aidhui"), 1);
  }

  for (size_t i = 0; i < components_.size(); i++) {
    components_[i]->subscribe();
  }
}

void UbloxNode::initializeRosDiagnostics() {
  for (size_t i = 0; i < components_.size(); i++) {
    components_[i]->initializeRosDiagnostics();
  }
}


void UbloxNode::processMonVer() {
  ublox_msgs::msg::MonVER monVer;
  if (!gps_.poll(monVer)) {
    throw std::runtime_error("Failed to poll MonVER & set relevant settings");
  }

  RCLCPP_DEBUG(get_logger(), "%s, HW VER: %s", monVer.sw_version.c_array(),
               monVer.hw_version.c_array());
  // Convert extension to vector of strings
  std::vector<std::string> extension;
  extension.reserve(monVer.extension.size());
  for (std::size_t i = 0; i < monVer.extension.size(); ++i) {
    RCLCPP_DEBUG(get_logger(), "%s", monVer.extension[i].field.c_array());
    // Find the end of the string (null character)
    unsigned char* end = std::find(monVer.extension[i].field.begin(),
          monVer.extension[i].field.end(), '\0');
    extension.push_back(std::string(monVer.extension[i].field.begin(), end));
  }

  // Get the protocol version
  for(std::size_t i = 0; i < extension.size(); ++i) {
    std::size_t found = extension[i].find("PROTVER");
    if (found != std::string::npos) {
      protocol_version_ = ::atof(
          extension[i].substr(8, extension[i].size()-8).c_str());
      break;
    }
  }
  if (protocol_version_ == 0) {
    RCLCPP_WARN(get_logger(), "Failed to parse MonVER and determine protocol version. %s",
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
      gnss.add(strs[i]);
    }
  } else {
    for (std::size_t i = 0; i < extension.size(); ++i) {
      std::vector<std::string> strs;
      // Up to 2nd to last line
      if (i <= extension.size() - 2) {
        strs = stringSplit(extension[i], "=");
        if(strs.size() > 1) {
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
          gnss.add(strs[i]);
        }
      }
    }
  }
}

bool UbloxNode::configureUblox() {
  try {
    if (!gps_.isInitialized()) {
      throw std::runtime_error("Failed to initialize.");
    }
    if (load_.load_mask != 0) {
      RCLCPP_DEBUG(get_logger(), "Loading u-blox configuration from memory. %u", load_.load_mask);
      if (!gps_.configure(load_)) {
        throw std::runtime_error(std::string("Failed to load configuration ") +
                                 "from memory");
      }
      if (load_.load_mask & load_.MASK_IO_PORT) {
        RCLCPP_DEBUG(get_logger(), "Loaded I/O configuration from memory, resetting serial %s",
          "communications.");
        gps_.reset(std::chrono::seconds(kResetWait));
        if (!gps_.isConfigured()) {
          throw std::runtime_error(std::string("Failed to reset serial I/O") +
            "after loading I/O configurations from device memory.");
        }
      }
    }

    if (config_on_startup_flag_) {
      if (set_usb_) {
        gps_.configUsb(usb_tx_, usb_in_, usb_out_);
      }
      if (!gps_.configRate(meas_rate, nav_rate)) {
        std::stringstream ss;
        ss << "Failed to set measurement rate to " << meas_rate
          << "ms and navigation rate to " << nav_rate;
        throw std::runtime_error(ss.str());
      }
      // If device doesn't have SBAS, will receive NACK (causes exception)
      if (gnss_.isSupported("SBAS")) {
        if (!gps_.configSbas(enable_sbas_, sbas_usage_, max_sbas_)) {
          throw std::runtime_error(std::string("Failed to ") +
                                  ((enable_sbas_) ? "enable" : "disable") +
                                  " SBAS.");
        }
      }
      if (!gps_.setPpp(enable_ppp_)) {
        throw std::runtime_error(std::string("Failed to ") +
                                ((enable_ppp_) ? "enable" : "disable")
                                + " PPP.");
      }
      if (!gps_.setDynamicModel(dmodel_)) {
        throw std::runtime_error("Failed to set model: " + dynamic_model_ + ".");
      }
      if (!gps_.setFixMode(fmode_)) {
        throw std::runtime_error("Failed to set fix mode: " + fix_mode_ + ".");
      }
      if (!gps_.setDeadReckonLimit(dr_limit_)) {
        std::stringstream ss;
        ss << "Failed to set dead reckoning limit: " << dr_limit_ << ".";
        throw std::runtime_error(ss.str());
      }
      if (set_dat_ && !gps_.configure(cfg_dat_)) {
        throw std::runtime_error("Failed to set user-defined datum.");
      }
      // Configure each component
      for (size_t i = 0; i < components_.size(); i++) {
        if (!components_[i]->configureUblox(gnss_)) {
          return false;
        }
      }
    }
    if (save_.save_mask != 0) {
      RCLCPP_DEBUG(get_logger(), "Saving the u-blox configuration, mask %u, device %u",
                   save_.save_mask, save_.device_mask);
      if (!gps_.configure(save_)) {
        RCLCPP_ERROR(get_logger(), "u-blox unable to save configuration to non-volatile memory");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Error configuring u-blox: %s", e.what());
    return false;
  }
  return true;
}

void UbloxNode::configureInf() {
  ublox_msgs::msg::CfgINF msg;
  // Subscribe to UBX INF messages
  ublox_msgs::msg::CfgINFBlock block;
  block.protocolID = block.PROTOCOL_ID_UBX;
  // Enable desired INF messages on each UBX port
  uint8_t mask = (inf_error_enable_ ? block.INF_MSG_ERROR : 0) |
                 (inf_warning_enable_ ? block.INF_MSG_WARNING : 0) |
                 (inf_notice_enable_ ? block.INF_MSG_NOTICE : 0) |
                 (inf_test_enable_ ? block.INF_MSG_TEST : 0) |
                 (inf_debug_enable_ ? block.INF_MSG_DEBUG : 0);
  for (size_t i = 0; i < block.inf_msg_mask.size(); i++) {
    block.inf_msg_mask[i] = mask;
  }

  msg.blocks.push_back(block);

  // IF NMEA is enabled
  if (uart_in_ & ublox_msgs::CfgPRT::PROTO_NMEA) {
    ublox_msgs::msg::CfgINFBlock block;
    block.protocolID = block.PROTOCOL_ID_NMEA;
    // Enable desired INF messages on each NMEA port
    for (size_t i = 0; i < block.inf_msg_mask.size(); i++) {
      block.infMsgMask[i] = mask;
    }
    msg.blocks.push_back(block);
  }

  ROS_DEBUG("Configuring INF messages");
  if (!gps_.configure(msg)) {
    RCLCPP_WARN(get_logger(), "Failed to configure INF messages");
  }
}

void UbloxNode::initializeIo() {
  gps_.setConfigOnStartup(config_on_startup_flag_);

  std::smatch match;
  if (std::regex_match(device_, match,
                       std::regex("(tcp|udp)://(.+):(\\d+)"))) {
    std::string proto(match[1]);
    if (proto == "tcp") {
      std::string host(match[2]);
      std::string port(match[3]);
      RCLCPP_INFO(get_logger(), "Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(),
                  port.c_str());
      gps_.initializeTcp(host, port);
    } else {
      throw std::runtime_error("Protocol '" + proto + "' is unsupported");
    }
  } else {
    gps_.initializeSerial(device_, baudrate_, uart_in_, uart_out_);
  }

  // raw data stream logging
  if (rawDataStreamPa_.isEnabled()) {
    gps_.setRawDataCallback(
      std::bind(&RawDataStreamPa::ubloxCallback,&rawDataStreamPa_, std::placeholders::_1, std::placeholders::_2));
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
    if (raw_data_enable_) {
      components_.push_back(std::make_unique<RawDataProduct>());
    }
  }
  // Must set firmware & hardware params before initializing diagnostics
  for (size_t i = 0; i < components_.size(); i++) {
    components_[i]->getRosParams(gnss_);
  }
  // Do this last
  initializeRosDiagnostics();

  if (configureUblox()) {
    RCLCPP_INFO(get_logger(), "U-Blox configured successfully.");
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
  if (gps_.isInitialized()) {
    gps_.close();
    RCLCPP_INFO(get_logger(), "Closed connection to %s.", device_.c_str());
  }
}

}
