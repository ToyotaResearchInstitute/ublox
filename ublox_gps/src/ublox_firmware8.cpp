#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <ublox_msgs/msg/cfg_gnss.hpp>
#include <ublox_msgs/msg/cfg_gnss_block.hpp>
#include <ublox_msgs/msg/mon_hw.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <ublox_msgs/msg/nav_sat.hpp>

#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

//
// Ublox Version 8
//
UbloxFirmware8::UbloxFirmware8(diagnostic_updater::Updater * updater) : UbloxFirmware7Plus<ublox_msgs::msg::NavPVT>(updater)
{
}

void UbloxFirmware8::getRosParams(const rclcpp::Node * node, const GNSS & gnss) {
  // UPD SOS configuration
  clear_bbr_ = node->declare_parameter("clear_bbr", false);
  bool save_on_shutdown = node->declare_parameter("save_on_shutdown", false);
  gps.setSaveOnShutdown(save_on_shutdown);

  // GNSS enable/disable
  enable_gps_ = node->declare_parameter("gnss.gps", true);
  enable_glonass_ = node->declare_parameter("gnss.glonass", false);
  enable_qzss_ = node->declare_parameter("gnss.qzss", false);
  qzss_sig_cfg_ = node->declare_parameter("gnss.qzss_sig_cfg", ublox_msgs::msg::CfgGNSSBlock::SIG_CFG_QZSS_L1CA);
  enable_sbas_ = node->declare_parameter("gnss.sbas", false);
  enable_galileo_ = node->declare_parameter("gnss.galileo", false);
  enable_beidou_ = node->declare_parameter("gnss.beidou", false);
  enable_imes_ = node->declare_parameter("gnss.imes", false);

  if (enable_gps_ && !gnss.isSupported("GPS")) {
    // ROS_WARN("gnss/gps is true, but GPS GNSS is not supported by %s",
    //          "this device");
  }
  if (enable_glonass_ && !gnss.isSupported("GLO")) {
    // ROS_WARN("gnss/glonass is true, but GLONASS is not supported by %s",
    //          "this device");
  }
  if (enable_galileo_ && !gnss.isSupported("GAL")) {
    // ROS_WARN("gnss/galileo is true, but Galileo GNSS is not supported %s",
    //          "by this device");
  }
  if (enable_beidou_ && !gnss.isSupported("BDS")) {
    // ROS_WARN("gnss/beidou is true, but Beidou GNSS is not supported %s",
    //          "by this device");
  }
  if (enable_imes_ && !gnss.isSupported("IMES")) {
    // ROS_WARN("gnss/imes is true, but IMES GNSS is not supported by %s",
    //          "this device");
  }
  if (enable_qzss_ && !gnss.isSupported("QZSS")) {
    // ROS_WARN("gnss/qzss is true, but QZSS is not supported by this device");
  }
  if (enable_sbas_ && !gnss.isSupported("SBAS")) {
    // ROS_WARN("gnss/sbas is true, but SBAS is not supported by this device");
  }

  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::msg::NavSatStatus::SERVICE_GPS
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS
      + (enable_beidou_ ? 1 : 0) * sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS
      + (enable_galileo_ ? 1 : 0) * sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

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
  bool temp = node->declare_parameter("nmea.limit82", false);
  cfg_nmea_.flags |= temp ? cfg_nmea_.FLAGS_LIMIT82 : 0;
  temp = node->declare_parameter("nmea.high_prec", false);
  cfg_nmea_.flags |= temp ? cfg_nmea_.FLAGS_HIGH_PREC : 0;
  // set filter
  temp = node->declare_parameter("nmea.filter.pos", false);
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
  temp = node->declare_parameter("nmea.gnssToFilter.beidou", false);
  cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_BEIDOU : 0;

  // TODO(clalancette): these two should be uint8, and are they required?
  cfg_nmea_.main_talker_id = node->declare_parameter("nmea.main_talker_id");
  cfg_nmea_.gsvTalkerId = node->declare_parameter("nmea.gsv_talker_id");

  // TODO(clalancette): reenable this
  // std::vector<uint8_t> bdsTalkerId;
  // getRosUint("nmea/bds_talker_id", bdsTalkerId);
  // cfg_nmea_.bdsTalkerId[0] = bdsTalkerId[0];
  // cfg_nmea_.bdsTalkerId[1] = bdsTalkerId[1];

  // Whether to publish Nav PVT messages
  bool nav_enabled = node->get_parameter("nav.publish.all");
  bool mon_hw_enabled = node->get_parameter("publish.mon.all");
  bool rxm_enabled = node->get_parameter("publish.rxm.all");
  nav_pvt_enabled_ = node->declare_parameter("publish.nav.pvt", nav_enabled);
  nav_sat_enabled_ = node->declare_parameter("publish.nav.sat", nav_enabled);
  mon_hw_enabled_ = node->declare_parameter("publish.mon.hw", mon_hw_enabled);
  rxm_rtcm_enabled_ = node->declare_parameter("publish.rxm.rtcm", rxm_enabled);
}

bool UbloxFirmware8::configureUblox(const GNSS & gnss) {
  (void)gnss;
  if (clear_bbr_) {
    // clear flash memory
    if (!gps.clearBbr()) {
      // ROS_ERROR("u-blox failed to clear flash memory");
    }
  }
  //
  // Configure the GNSS, only if the configuration is different
  //
  // First, get the current GNSS configuration
  ublox_msgs::msg::CfgGNSS cfg_gnss;
  if (gps.poll(cfg_gnss)) {
    // ROS_DEBUG("Read GNSS config.");
    // ROS_DEBUG("Num. tracking channels in hardware: %i", cfg_gnss.numTrkChHw);
    // ROS_DEBUG("Num. tracking channels to use: %i", cfg_gnss.numTrkChUse);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  // Then, check the configuration for each GNSS. If it is different, change it.
  bool correct = true;
  for (size_t i = 0; i < cfg_gnss.blocks.size(); i++) {
    ublox_msgs::msg::CfgGNSSBlock block = cfg_gnss.blocks[i];
    if (block.gnss_id == block.GNSS_ID_GPS
        && enable_gps_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_gps_;
      // ROS_DEBUG("GPS Configuration is different");
    } else if (block.gnss_id == block.GNSS_ID_SBAS
               && enable_sbas_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_sbas_;
      // ROS_DEBUG("SBAS Configuration is different");
    } else if (block.gnss_id == block.GNSS_ID_GALILEO
               && enable_galileo_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_galileo_;
      // ROS_DEBUG("Galileo GNSS Configuration is different");
    } else if (block.gnss_id == block.GNSS_ID_BEIDOU
               && enable_beidou_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_beidou_;
      // ROS_DEBUG("BeiDou Configuration is different");
    } else if (block.gnss_id == block.GNSS_ID_IMES
               && enable_imes_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_imes_;
    } else if (block.gnss_id == block.GNSS_ID_QZSS
               && (enable_qzss_ != (block.flags & block.FLAGS_ENABLE)
               || (enable_qzss_
               && qzss_sig_cfg_ != (block.flags & block.FLAGS_SIG_CFG_MASK)))) {
      // ROS_DEBUG("QZSS Configuration is different %u, %u",
      //           block.flags & block.FLAGS_ENABLE,
      //           enable_qzss_);
      correct = false;
      // ROS_DEBUG("QZSS Configuration: %u", block.flags);
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_qzss_;
      // ROS_DEBUG("QZSS Configuration: %u", cfg_gnss.blocks[i].flags);
      if (enable_qzss_) {
        // Only change sig cfg if enabling
        cfg_gnss.blocks[i].flags |= qzss_sig_cfg_;
      }
    } else if (block.gnss_id == block.GNSS_ID_GLONASS
               && enable_glonass_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_glonass_;
      // ROS_DEBUG("GLONASS Configuration is different");
    }
  }

  // If the GNSS is already configured correctly, do not re-configure GNSS
  // since this requires a cold reset
  if (correct) {
    // ROS_DEBUG("U-Blox GNSS configuration is correct. GNSS not re-configured.");
  } else if (!gps.configGnss(cfg_gnss, std::chrono::seconds(15))) {
    throw std::runtime_error(std::string("Failed to cold reset device ") +
                             "after configuring GNSS");
  }

  //
  // NMEA config
  //
  if (set_nmea_ && !gps.configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware8::subscribe(const rclcpp::Node * node) {
  // Subscribe to Nav PVT
  gps.subscribe<ublox_msgs::msg::NavPVT>(
    std::bind(&UbloxFirmware7Plus::callbackNavPvt, this, std::placeholders::_1), 1);

  // Subscribe to Nav SAT messages
  if (nav_sat_enabled_) {
    gps.subscribe<ublox_msgs::msg::NavSAT>(std::bind(
        publish<ublox_msgs::msg::NavSAT>, std::placeholders::_1, "navsat"), 20);
  }

  // Subscribe to Mon HW
  if (mon_hw_enabled_) {
    gps.subscribe<ublox_msgs::msg::MonHW>(std::bind(
        publish<ublox_msgs::msg::MonHW>, std::placeholoders::_1, "monhw"), 1);
  }

  // Subscribe to RTCM messages
  if (rxm_rtcm_enabled_) {
    gps.subscribe<ublox_msgs::msg::RxmRTCM>(std::bind(
        publish<ublox_msgs::msg::RxmRTCM>, std::placeholders::_1, "rxmrtcm"), 1);
  }
}

}
