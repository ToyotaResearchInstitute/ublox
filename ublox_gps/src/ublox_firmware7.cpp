#include <functional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <ublox_msgs/msg/cfg_gnss.hpp>
#include <ublox_msgs/msg/cfg_gnss_block.hpp>
#include <ublox_msgs/msg/mon_hw.hpp>
#include <ublox_msgs/msg/nav_sv_info.hpp>

#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

//
// Ublox Firmware Version 7
//
UbloxFirmware7::UbloxFirmware7(diagnostic_updater::Updater * updater) : UbloxFirmware7Plus<ublox_msgs::msg::NavPVT7>(updater)
{
}

void UbloxFirmware7::getRosParams(const rclcpp::Node * node, const GNSS & gnss) {
  //
  // GNSS configuration
  //
  // GNSS enable/disable
  enable_gps_ = node->declare_parameter("gnss.gps", true);
  enable_glonass_ = node->declare_parameter("gnss.glonass", false);
  enable_qzss_ = node->declare_parameter("gnss.qzss", false);
  qzss_sig_cfg_ = node->declare_parameter("gnss.qzss_sig_cfg", ublox_msgs::msg::CfgGNSSBlock::SIG_CFG_QZSS_L1CA);
  enable_sbas_ = node->declare_parameter("gnss.sbas", false);

  if (enable_gps_ && !gnss.isSupported("GPS")) {
    // ROS_WARN("gnss/gps is true, but GPS GNSS is not supported by this device");
  }
  if (enable_glonass_ && !gnss.isSupported("GLO")) {
    // ROS_WARN("gnss/glonass is true, but GLONASS is not %s",
    //          "supported by this device");
  }
  if (enable_qzss_ && !gnss.isSupported("QZSS")) {
    // ROS_WARN("gnss/qzss is true, but QZSS is not supported by this device");
  }
  if (enable_sbas_ && !gnss.isSupported("SBAS")) {
    // ROS_WARN("gnss/sbas is true, but SBAS is not supported by this device");
  }

  if (node->has_parameter("gnss.galileo")) {
    // ROS_WARN("ublox_version < 8, ignoring Galileo GNSS Settings");
  }
  if (node->has_parameter("gnss.beidou")) {
    // ROS_WARN("ublox_version < 8, ignoring BeiDou Settings");
  }
  if (node->has_parameter("gnss.imes")) {
    // ROS_WARN("ublox_version < 8, ignoring IMES GNSS Settings");
  }

  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::msg::NavSatStatus::SERVICE_GPS
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

  //
  // NMEA Configuration
  //
  set_nmea_ = node->declare_parameter("nmea.set", false);
  if (!set_nmea_) {
    return;
  }

  node->declare_parameter("nmea.version");
  rclcpp::Parameter version = node->get_parameter("nmea.version");
  if (version.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
      "true, therefore nmea/version must be set");
  }
  // TODO(clalancette): This should be a uint8_t; check the range here.
  cfg_nmea_.version = version.get_value<int>();

  node->declare_parameter("nmea.num_sv");
  rclcpp::Parameter num_sv = node->get_parameter("nmea.num_sv");
  if (num_sv.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
      "true, therefore nmea/num_sv must be set");
  }
  // TODO(clalancette): This should be a uint8_t; check the range here.
  cfg_nmea_.num_sv = num_sv.get_value<int>();

  node->declare_parameter("nmea.sv_numbering");
  rclcpp::Parameter sv_numbering = node->get_parameter("nmea.sv_numbering");
  if (sv_numbering.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
      "true, therefore nmea/sv_numbering must be set");
  }
  // TODO(clalancette): This should be a uint8_t; check the range here.
  cfg_nmea_.sv_numbering = sv_numbering.get_value<int>();

  node->declare_parameter("nmea.compat");
  rclcpp::Parameter nmea_compat = node->get_parameter("nmea.compat");
  if (nmea_compat.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
      "true, therefore nmea/compat must be set");
  }
  bool compat = nmea_compat.get_value<bool>();

  node->declare_parameter("nmea.consider");
  rclcpp::Parameter nmea_consider = node->get_parameter("nmea.consider");
  if (nmea_consider.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
      "true, therefore nmea/consider must be set");
  }
  bool consider = nmea_consider.get_value<bool>();

  // set flags
  cfg_nmea_.flags = compat ? cfg_nmea_.FLAGS_COMPAT : 0;
  cfg_nmea_.flags |= consider ? cfg_nmea_.FLAGS_CONSIDER : 0;
  // set filter
  bool temp = node->declare_parameter("nmea.filter.pos", false);
  cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_POS : 0;
  temp = node->declare_parameter("nmea.filter.msk_pos", false);
  cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_MSK_POS : 0;
  temp = node->declare_parameter("nmea.filter.time", false);
  cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TIME : 0;
  temp = node->declare_parameter("nmea.filter.date", false);
  cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_DATE : 0;
  temp = node->declare_parameter("nmea.filter.gps_only", false);
  cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_GPS_ONLY : 0;
  temp = node->declare_parameter("nmea.filter.track", false);
  cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TRACK : 0;
  // set gnssToFilter
  temp = node->declare_parameter("nmea.gnssToFilter.gps", false);
  cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_GPS : 0;
  temp = node->declare_parameter("nmea.gnssToFilter.sbas", false);
  cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_SBAS : 0;
  temp = node->declare_parameter("nmea.gnssToFilter.qzss", false);
  cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_QZSS : 0;
  temp = node->declare_parameter("nmea.gnssToFilter.glonass", false);
  cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_GLONASS : 0;

  // TODO(clalancette): these two should be uint8, and are they required?
  cfg_nmea_.main_talker_id = node->declare_parameter("nmea.main_talker_id");
  cfg_nmea_.gsvTalkerId = node->declare_parameter("nmea.gsv_talker_id");

  // Whether to publish Nav PVT messages to a ROS topic
  bool nav_enabled = node->get_parameter("publish.nav.all");
  enable_nav_pvt_ = node->declare_parameter("publish.nav.pvt", nav_enabled);
  enable_nav_svinfo_ = node->declare_parameter("publish.nav.svinfo", nav_enabled);
  bool mon_enabled = node->get_parameter("mon.all");
  enable_mon_hw_ = node->declare_parameter("publish.mon_hw", mon_enabled);
}

bool UbloxFirmware7::configureUblox(const GNSS & gnss) {
  /** Configure the GNSS **/
  ublox_msgs::msg::CfgGNSS cfgGNSSRead;
  if (gps.poll(cfgGNSSRead)) {
    // ROS_DEBUG("Read GNSS config.");
    // ROS_DEBUG("Num. tracking channels in hardware: %i", cfgGNSSRead.numTrkChHw);
    // ROS_DEBUG("Num. tracking channels to use: %i", cfgGNSSRead.numTrkChUse);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  ublox_msgs::msg::CfgGNSS cfgGNSSWrite;
  cfgGNSSWrite.num_config_blocks = 1;  // do services one by one
  cfgGNSSWrite.num_trk_ch_hw = cfgGNSSRead.num_trk_ch_hw;
  cfgGNSSWrite.num_trk_ch_use = cfgGNSSRead.num_trk_ch_use;
  cfgGNSSWrite.msg_ver = 0;

  // configure GLONASS
  if (gnss.isSupported("GLO")) {
    ublox_msgs::msg::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_GLONASS;
    block.res_trk_ch = block.RES_TRK_CH_GLONASS;
    block.max_trk_ch = block.MAX_TRK_CH_GLONASS;
    block.flags = enable_glonass_ ? block.SIG_CFG_GLONASS_L1OF : 0;
    cfgGNSSWrite.blocks.push_back(block);
    if (!gps.configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_glonass_) ? "enable" : "disable") +
                               " GLONASS.");
    }
  }

  if (gnss.isSupported("QZSS")) {
    // configure QZSS
    ublox_msgs::msg::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_QZSS;
    block.res_trk_ch = block.RES_TRK_CH_QZSS;
    block.max_trk_ch = block.MAX_TRK_CH_QZSS;
    block.flags = enable_qzss_ ? qzss_sig_cfg_ : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps.configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_glonass_) ? "enable" : "disable") +
                               " QZSS.");
    }
  }

  if (gnss.isSupported("SBAS")) {
    // configure SBAS
    ublox_msgs::msg::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_SBAS;
    block.res_trk_ch = block.RES_TRK_CH_SBAS;
    block.max_trk_ch = block.MAX_TRK_CH_SBAS;
    block.flags = enable_sbas_ ? block.SIG_CFG_SBAS_L1CA : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps.configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_sbas_) ? "enable" : "disable") +
                               " SBAS.");
    }
  }

  if (set_nmea_ && !gps.configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware7::subscribe(const rclcpp::Node * node) {
  // Subscribe to Nav PVT (always does so since fix information is published
  // from this)
  gps.subscribe<ublox_msgs::msg::NavPVT7>(std::bind(
        &UbloxFirmware7Plus::callbackNavPvt, this, std::placeholders::_1),
        1);

  // Subscribe to Nav SVINFO
  if (nav_svinfo_enable_) {
    gps.subscribe<ublox_msgs::msg::NavSVINFO>(std::bind(
        publish<ublox_msgs::msg::NavSVINFO>, std::placeholders::_1, "navsvinfo"),
        20);
  }

  // Subscribe to Mon HW
  if (mon_hw_enable_) {
    gps.subscribe<ublox_msgs::msg::MonHW>(std::bind(
        publish<ublox_msgs::msg::MonHW>, std::placeholders::_1, "monhw"), 1);
  }
}

}
