#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_tmode3.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <ublox_msgs/msg/nav_svin.hpp>

#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

//! Default measurement period for HPG devices
constexpr static uint16_t kDefaultMeasPeriod = 250;

//
// u-blox High Precision GNSS Reference Station
//
void HpgRefProduct::getRosParams(const rclcpp::Node * node, const GNSS & gnss) {
  (void)gnss;
  if (node->get_parameter("config_on_startup")) {
    if (nav_rate * meas_rate != 1000) {
      // ROS_WARN("For HPG Ref devices, nav_rate should be exactly 1 Hz.");
    }

    node->declare_parameter("tmode3");
    rclcpp::Parameter tmode3 = node->get_parameter("tmode3");
    if (tmode3.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      throw std::runtime_error("Invalid settings: TMODE3 must be set");
    }
    // TODO(clalancette): this should be a uint8_t; check the range here
    tmode3_ = tmode3.get_value<uint8_t>();

    if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_FIXED) {
      // TODO(clalancette): Figure out how to do vectors with parameters
      node->declare_parameter("arp.position");
      rclcpp::Parameter arp_position = node->get_parameter("arp.position");
      if (arp_position.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        throw std::runtime_error(std::string("Invalid settings: arp/position ")
                                + "must be set if TMODE3 is fixed");
      }
      // TODO(clalancette): this should be a uint8_t; check the range here
      arp_position_ = arp_position.get_value<uint8_t>();

      // TODO(clalancette): Figure out how to do vectors with parameters
      node->declare_parameter("arp.position_hp");
      rclcpp::Parameter arp_position_hp = node->get_parameter("arp.position_hp");
      if (arp_position_hp.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        throw std::runtime_error(std::string("Invalid settings: arp/position_hp ")
                                + "must be set if TMODE3 is fixed");
      }
      // TODO(clalancette): this should be an int8_t; check the range here
      arp_position_hp_ = arp_position_hp.get_value<uint8_t>();

      node->declare_parameter("arp.acc");
      rclcpp::Parameter fixed_pos_acc = node->get_parameter("arp.acc");
      if (fixed_pos_acc.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        throw std::runtime_error(std::string("Invalid settings: arp/acc ")
                                + "must be set if TMODE3 is fixed");
      }
      fixed_pos_acc_ = fixed_pos_acc.get_value<float>();

      lla_flag_ = node->declare_parameter("arp.lla_flag", false);
      rclcpp::Parameter lla_flag = node->get_parameter("arp.lla_flag");
      if (lla_flag.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        // ROS_WARN("arp/lla_flag param not set, assuming ARP coordinates are %s",
        //         "in ECEF");
        lla_flag_ = false;
      } else {
        lla_flag_ = lla_flag.get_value<bool>();
      }
    } else if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
      svin_reset_ = node->declare_parameter("sv_in.reset", true);

      node->declare_parameter("sv_in.min_dur");
      rclcpp::Parameter min_dur = node->get_parameter("sv_in.min_dur");
      if (min_dur.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        throw std::runtime_error(std::string("Invalid settings: sv_in/min_dur ")
                                + "must be set if TMODE3 is survey-in");
      }
      sv_in_min_dur_ = min_dur.get_value<uint32_t>();

      node->declare_parameter("sv_in.acc_lim");
      rclcpp::Parameter acc_lim = node->get_parameter("sv_in.acc_lim");
      if (acc_lim.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        throw std::runtime_error(std::string("Invalid settings: sv_in/acc_lim ")
                                + "must be set if TMODE3 is survey-in");
      }
      sv_in_acc_lim_ = acc_lim.get_value<float>();
    } else if(tmode3_ != ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_DISABLED) {
      throw std::runtime_error(std::string("tmode3 param invalid. See CfgTMODE3")
                              + " flag constants for possible values.");
    }
  }

  bool nav_enable = node->get_parameter("publish.nav.all", false);
  nav_svin_enable_ = node->declare_parameter("publish.nav.svin", nav_enable);
}

bool HpgRefProduct::configureUblox(const GNSS & gnss) {
  (void)gnss;
  // Configure TMODE3
  if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_DISABLED) {
    if (!gps.disableTmode3()) {
      throw std::runtime_error("Failed to disable TMODE3.");
    }
    mode_ = DISABLED;
  } else if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_FIXED) {
    if (!gps.configTmode3Fixed(lla_flag_, arp_position_, arp_position_hp_,
                               fixed_pos_acc_)) {
      throw std::runtime_error("Failed to set TMODE3 to fixed.");
    }
    if (!gps.configRtcm(rtcm_ids, rtcm_rates)) {
      throw std::runtime_error("Failed to set RTCM rates");
    }
    mode_ = FIXED;
  } else if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
    if (!svin_reset_) {
      ublox_msgs::msg::NavSVIN nav_svin;
      if (!gps.poll(nav_svin)) {
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
        setTimeMode();
        return true;
      }
      ublox_msgs::msg::NavPVT nav_pvt;
      if (!gps.poll(nav_pvt)) {
        throw std::runtime_error(std::string("Failed to poll NavPVT while") +
                                 " configuring survey-in");
      }
      // Don't reset survey in if in time mode with a good fix
      if (nav_pvt.fixType == nav_pvt.FIX_TYPE_TIME_ONLY
          && nav_pvt.flags & nav_pvt.FLAGS_GNSS_FIX_OK) {
        setTimeMode();
        return true;
      }
    }
    // Reset the Survey In
    // For Survey in, meas rate must be at least 1 Hz
    uint16_t meas_rate_temp = meas_rate < 1000 ? meas_rate : 1000; // [ms]
    // If measurement period isn't a factor of 1000, set to default
    if (1000 % meas_rate_temp != 0) {
      meas_rate_temp = kDefaultMeasPeriod;
    }
    // Set nav rate to 1 Hz during survey in
    if (!gps.configRate(meas_rate_temp, (int) 1000 / meas_rate_temp)) {
      throw std::runtime_error(std::string("Failed to set nav rate to 1 Hz") +
                               "before setting TMODE3 to survey-in.");
    }
    // As recommended in the documentation, first disable, then set to survey in
    if (!gps.disableTmode3()) {
      // ROS_ERROR("Failed to disable TMODE3 before setting to survey-in.");
    } else {
      mode_ = DISABLED;
    }
    // Set to Survey in mode
    if (!gps.configTmode3SurveyIn(sv_in_min_dur_, sv_in_acc_lim_)) {
      throw std::runtime_error("Failed to set TMODE3 to survey-in.");
    }
    mode_ = SURVEY_IN;
  }
  return true;
}

void HpgRefProduct::subscribe(const rclcpp::Node * node) {
  // Subscribe to Nav Survey-In
  gps.subscribe<ublox_msgs::msg::NavSVIN>(std::bind(
      &HpgRefProduct::callbackNavSvIn, this, std::placeholders::_1), 1);

  if (nav_svin_enable_) {
    nav_svin_pub_ = node->create_publisher<ublox_msgs::msg::NavSVIN>("navsvin", 1);
  }
}

void HpgRefProduct::callbackNavSvIn(ublox_msgs::msg::NavSVIN m) {
  if (nav_svin_enable_) {
    nav_svin_pub_->publish(m);
  }

  last_nav_svin_ = m;

  if (!m.active && m.valid && mode_ == SURVEY_IN) {
    setTimeMode();
  }

  updater_->update();
}

bool HpgRefProduct::setTimeMode() {
  // ROS_INFO("Setting mode (internal state) to Time Mode");
  mode_ = TIME;

  // Set the Measurement & nav rate to user config
  // (survey-in sets nav_rate to 1 Hz regardless of user setting)
  if (!gps.configRate(meas_rate, nav_rate)) {
    // ROS_ERROR("Failed to set measurement rate to %d ms %s %d", meas_rate,
    //           "navigation rate to ", nav_rate);
  }
  // Enable the RTCM out messages
  if (!gps.configRtcm(rtcm_ids, rtcm_rates)) {
    // ROS_ERROR("Failed to configure RTCM IDs");
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
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "Not configured";
  } else if (mode_ == DISABLED){
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "Disabled";
  } else if (mode_ == SURVEY_IN) {
    if (!last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      stat.message = "Survey-In inactive and invalid";
    } else if (last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "Survey-In active but invalid";
    } else if (!last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "Survey-In complete";
    } else if (last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "Survey-In active and valid";
    }

    stat.add("iTOW [ms]", last_nav_svin_.i_tow);
    stat.add("Duration [s]", last_nav_svin_.dur);
    stat.add("# observations", last_nav_svin_.obs);
    stat.add("Mean X [m]", last_nav_svin_.mean_x * 1e-2);
    stat.add("Mean Y [m]", last_nav_svin_.mean_y * 1e-2);
    stat.add("Mean Z [m]", last_nav_svin_.mean_z * 1e-2);
    stat.add("Mean X HP [m]", last_nav_svin_.mean_x_hp * 1e-4);
    stat.add("Mean Y HP [m]", last_nav_svin_.mean_y_hp * 1e-4);
    stat.add("Mean Z HP [m]", last_nav_svin_.mean_z_hp * 1e-4);
    stat.add("Mean Accuracy [m]", last_nav_svin_.mean_acc * 1e-4);
  } else if (mode_ == FIXED) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "Fixed Position";
  } else if (mode_ == TIME) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "Time";
  }
}

}
