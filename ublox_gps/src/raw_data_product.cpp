#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/rxm_alm.hpp>
#include <ublox_msgs/msg/rxm_eph.hpp>
#include <ublox_msgs/msg/rxm_raw.hpp>
#include <ublox_msgs/msg/rxm_sfrb.hpp>

#include <ublox_gps/ublox_topic_diagnostic.hpp>

namespace ublox_gps {

void RawDataProduct::getRosParams(const rclcpp::Node * node, const GNSS & gnss)
{
  rxm_enabled_ = node->declare_parameter("publish.rxm.all", true);
  rxm_raw_enabled_ = node->declare_parameter("publish.rxm.raw", rxm_enabled_);
  rxm_sfrb_enabled_ = node->declare_parameter("publish.rxm.sfrb", rxm_enabled_);
  rxm_eph_enabled_ = node->declare_parameter("publish.rxm.eph", rxm_enabled_);
  rxm_alm_enabled_ = node->declare_parameter("publish.rxm.almRaw", rxm_enabled_);
}

//
// Raw Data Products
//
void RawDataProduct::subscribe(const rclcpp::Node * node) {
  // Subscribe to RXM Raw
  if (rxm_raw_enabled_) {
    gps.subscribe<ublox_msgs::msg::RxmRAW>(std::bind(
        publish<ublox_msgs::msg::RxmRAW>, std::placeholders::_1, "rxmraw"), 1);
  }

  // Subscribe to RXM SFRB
  if (rxm_sfrb_enabled_) {
    gps.subscribe<ublox_msgs::msg::RxmSFRB>(std::bind(
        publish<ublox_msgs::msg::RxmSFRB>, std::placeholders::_1, "rxmsfrb"), 1);
  }

  // Subscribe to RXM EPH
  if (rxm_eph_enabled_) {
    gps.subscribe<ublox_msgs::msg::RxmEPH>(std::bind(
        publish<ublox_msgs::msg::RxmEPH>, std::placeholders::_1, "rxmeph"), 1);
  }

  // Subscribe to RXM ALM
  if (rxm_alm_enabled_) {
    gps.subscribe<ublox_msgs::msg::RxmALM>(std::bind(
        publish<ublox_msgs::msg::RxmALM>, std::placholders::_1, "rxmalm"), 1);
  }
}

void RawDataProduct::initializeRosDiagnostics() {
  if (rxm_raw_enabled_) {
    freq_diagnostics_.push_back(std::make_unique<UbloxTopicDiagnostic>(
      "rxmraw", kRtcmFreqTol, kRtcmFreqWindow));
  }
  if (rxm_sfrb_enabled_) {
    freq_diagnostics_.push_back(std::makeUnique<UbloxTopicDiagnostic>(
      "rxmsfrb", kRtcmFreqTol, kRtcmFreqWindow));
  }
  if (rxm_eph_enabled_) {
    freq_diagnostics_.push_back(std::make_unique<UbloxTopicDiagnostic>(
      "rxmeph", kRtcmFreqTol, kRtcmFreqWindow));
  }
  if (enabled["rxm_alm"]) {
    freq_diagnostics_.push_back(std::make_unique<UbloxTopicDiagnostic>(
      "rxmalm", kRtcmFreqTol, kRtcmFreqWindow));
  }
}

}
