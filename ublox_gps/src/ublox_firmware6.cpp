#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <ublox_msgs/msg/mon_hw6.hpp>
#include <ublox_msgs/msg/nav_pos_llh.hpp>
#include <ublox_msgs/msg/nav_sol.hpp>
#include <ublox_msgs/msg/nav_svinfo.hpp>
#include <ublox_msgs/msg/nav_vel_ned.hpp>

#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

//
// U-Blox Firmware Version 6
//
UbloxFirmware6::UbloxFirmware6(diagnostic_updater::Updater * updater) : UbloxFirmware(updater)
{
}

void UbloxFirmware6::getRosParams(const rclcpp::Node * node, const GNSS & gnss) {
  (void)gnss;
  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

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
  temp = node->declare_parameter("nmea.filter.sbas", false);
  cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_SBAS_FILT : 0;
  temp = node->declare_parameter("nmea.filter.track", false);
  cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TRACK : 0;

  // Whether or not to publish Nav POS LLH (always subscribes)
  bool nav_enabled = node->get_parameter("publish.nav.all");
  nav_pos_llh_enabled_ = node->declare_parameter("publish.nav.posllh", nav_enabled);
  nav_sol_enabled_ = node->declare_parameter("publish.nav.sol", nav_enabled);
  nav_velned_enabled_ = node->declare_parameter("publish.nav.velned", nav_enabled);
  nav_svinfo_enabled_ = node->declare_parameter("publish.nav.svinfo", nav_enabled);
  bool mon_enabled = node->get_parameter("publish.mon.all");
  mon_hw_enabled_ = node->declare_parameter("publish.mon_hw", mon_enabled);
}

bool UbloxFirmware6::configureUblox(const GNSS & gnss) {
  (void)gnss;
  // ROS_WARN("ublox_version < 7, ignoring GNSS settings");

  if (set_nmea_ && !gps.configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware6::subscribe(const rclcpp::Node * node) {
  // Always subscribes to these messages, but may not publish to ROS topic
  // Subscribe to Nav POSLLH
  gps.subscribe<ublox_msgs::msg::NavPOSLLH>(std::bind(
      &UbloxFirmware6::callbackNavPosLlh, this, std::placeholders::_1), 1);
  gps.subscribe<ublox_msgs::msg::NavSOL>(std::bind(
  // Subscribe to Nav SOL
      &UbloxFirmware6::callbackNavSol, this, std::placeholders::_1), 1);
  // Subscribe to Nav VELNED
  gps.subscribe<ublox_msgs::msg::NavVELNED>(std::bind(
      &UbloxFirmware6::callbackNavVelNed, this, std::placeholders::_1), 1);

  // Subscribe to Nav SVINFO
  if (nav_svinfo_enabled_) {
    gps.subscribe<ublox_msgs::msg::NavSVINFO>(std::bind(
        publish<ublox_msgs::msg::NavSVINFO>, std::placeholders::_1, "navsvinfo"),
        20);
  }

  // Subscribe to Mon HW
  if (mon_hw_enabled_) {
    gps.subscribe<ublox_msgs::msg::MonHW6>(std::bind(
        publish<ublox_msgs::msg::MonHW6>, std::placeholders::_1, "monhw"), 1);
  }

  nav_pos_llh_pub_ = node->create_publisher<ublox_msgs::msg::NavPOSLLH>("navposllh", 1);
  fix_pub_ = node->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);
  vel_ned_pub_ = node->create_publisher<ublox_msgs::msg::NavVELNED>("navvelned", 1);
  velocity_pub_ = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("fix_velocity", 1);
  nav_sol_pub_ = node->create_publisher<ublox_msgs::msg::NavSOL>("navsol", 1);
}

void UbloxFirmware6::fixDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  // Set the diagnostic level based on the fix status
  if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "2D fix";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "3D fix";
  } else if (last_nav_sol_.gps_fix ==
             ublox_msgs::msg::NavSOL::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "Time fix only";
  }
  // If fix is not ok (within DOP & Accuracy Masks), raise the diagnostic level
  if (!(last_nav_sol_.flags & ublox_msgs::msg::NavSOL::FLAGS_GPS_FIX_OK)) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message += ", fix not ok";
  }
  // Raise diagnostic level to error if no fix
  if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
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
  if (nav_posllh_enabled_) {
    nav_pos_llh_pub_->publish(m);
  }

  // Position message
  builtin_interfaces::msg::Time fix_header_stamp;
  if (m.i_tow == last_nav_vel_.i_tow) {
    fix_header_stamp = velocity_.header.stamp; // use last timestamp
  } else {
    fix_header_stamp = rclcpp::Time::now(); // new timestamp
  }
  fix_.header.stamp = fix_header_stamp;

  fix_.header.frame_id = frame_id;
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
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix_.status.service = fix_.status.SERVICE_GPS;
  fix_pub_->publish(fix_);
  last_nav_pos_ = m;
  // update diagnostics
  freq_diag_->diagnostic->tick(fix_header_stamp);
  updater_->update();
}

void UbloxFirmware6::callbackNavVelNed(const ublox_msgs::msg::NavVELNED& m) {
  if (nav_velned_enabled_) {
    vel_ned_pub->publish(m);
  }

  // Example geometry message
  if (m.i_tow == last_nav_pos_.i_tow) {
    velocity_.header.stamp = fix_.header.stamp; // same time as last navposllh
  } else {
    velocity_.header.stamp = rclcpp::Time::now(); // create a new timestamp
  }
  velocity_.header.frame_id = frame_id;

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

  velocity_pub_->publish(velocity_);
  last_nav_vel_ = m;
}

void UbloxFirmware6::callbackNavSol(const ublox_msgs::msg::NavSOL& m) {
  if (nav_sol_enabled_) {
    nav_sol_pub_->publish(m);
  }
  last_nav_sol_ = m;
}

}
