#include <functional>
#include <stdexcept>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_dgnss.hpp>
#include <ublox_msgs/msg/nav_rel_pos_ned.hpp>

#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

HpgRovProduct::HpgRovProduct(diagnostic_updater::Updater * updater) : updater_(updater)
{
}

//
// U-Blox High Precision GNSS Rover
//
void HpgRovProduct::getRosParams(const rclcpp::Node * node, const GNSS & gnss) {
  (void)gnss;
  // default to float, see CfgDGNSS message for details
  dngss_mode_ = node->declare_parameter("dgnss_mode", ublox_msgs::msg::CfgDGNSS:DGNSS_MODE_RTK_FIXED);
  bool nav_enabled = node->get_parameter("publish.nav.all");
  // Whether to publish Nav Relative Position NED
  nav_relposned_enabled_ = node->declare_parameter("publish.nav.relposned", nav_enabled);
}

bool HpgRovProduct::configureUblox(const GNSS & gnss) {
  (void)gnss;
  // Configure the DGNSS
  if (!gps.setDgnss(dgnss_mode_)) {
    throw std::runtime_error(std::string("Failed to Configure DGNSS"));
  }
  return true;
}

void HpgRovProduct::subscribe(const rclcpp::Node * node) {
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  gps.subscribe<ublox_msgs::msg::NavRELPOSNED>(std::bind(
     &HpgRovProduct::callbackNavRelPosNed, this, std::placeholders::_1), 1);

  publisher_ = node->create_publisher<ublox_msgs::msg::NavRELPOSNED>("navrelposned", 1);
}

void HpgRovProduct::initializeRosDiagnostics() {
  freq_rtcm_ = UbloxTopicDiagnostic(std::string("rxmrtcm"),
                                    kRtcmFreqMin, kRtcmFreqMax,
                                    kRtcmFreqTol, kRtcmFreqWindow);
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
  if (nav_relposned_enabled_) {
    publisher->publish(m);
  }

  last_rel_pos_ = m;
  updater_->update();
}

}
