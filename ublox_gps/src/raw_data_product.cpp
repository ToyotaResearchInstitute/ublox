#include <memory>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <ublox_msgs/RxmALM.h>
#include <ublox_msgs/RxmEPH.h>
#include <ublox_msgs/RxmRAW.h>
#include <ublox_msgs/RxmSFRB.h>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/raw_data_product.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// Raw Data Products
//
RawDataProduct::RawDataProduct(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, ros::NodeHandle* node)
  : nav_rate_(nav_rate), meas_rate_(meas_rate), updater_(updater), node_(node) {
  rxm_raw_pub_ = node_->advertise<ublox_msgs::RxmRAW>("rxmraw", 1);
  rxm_sfrb_pub_ = node_->advertise<ublox_msgs::RxmSFRB>("rxmsfrb", 1);
  rxm_eph_pub_ = node_->advertise<ublox_msgs::RxmEPH>("rxmeph", 1);
  rxm_alm_pub_ = node_->advertise<ublox_msgs::RxmALM>("rxmalm", 1);
}

void RawDataProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to RXM Raw
  if (getRosBoolean(node_, "publish/rxm/raw")) {
    gps->subscribe<ublox_msgs::RxmRAW>([this](const ublox_msgs::RxmRAW &m) { rxm_raw_pub_.publish(m); },
                                       1);
  }

  // Subscribe to RXM SFRB
  if (getRosBoolean(node_, "publish/rxm/sfrb")) {
    gps->subscribe<ublox_msgs::RxmSFRB>([this](const ublox_msgs::RxmSFRB &m) { rxm_sfrb_pub_.publish(m); },
                                        1);
  }

  // Subscribe to RXM EPH
  if (getRosBoolean(node_, "publish/rxm/eph")) {
    gps->subscribe<ublox_msgs::RxmEPH>([this](const ublox_msgs::RxmEPH &m) { rxm_eph_pub_.publish(m); },
                                       1);
  }

  // Subscribe to RXM ALM
  if (getRosBoolean(node_, "publish/rxm/almRaw")) {
    gps->subscribe<ublox_msgs::RxmALM>([this](const ublox_msgs::RxmALM &m) { rxm_alm_pub_.publish(m); },
                                       1);
  }
}

void RawDataProduct::initializeRosDiagnostics() {
  if (getRosBoolean(node_, "publish/rxm/raw")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmraw", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
  if (getRosBoolean(node_, "publish/rxm/sfrb")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmsfrb", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
  if (getRosBoolean(node_, "publish/rxm/eph")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmeph", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
  if (getRosBoolean(node_, "publish/rxm/almRaw")) {
    freq_diagnostics_.push_back(std::make_shared<UbloxTopicDiagnostic>(
      "rxmalm", kRtcmFreqTol, kRtcmFreqWindow, nav_rate_, meas_rate_, updater_));
  }
}

}  // namespace ublox_node