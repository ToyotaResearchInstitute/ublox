#include <functional>
#include <stdexcept>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/rxm_rawx.hpp>
#include <ublox_msgs/msg/rxm_sfrbx.hpp>
#include <ublox_msgs/msg/tim_tm2.hpp>

#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

TimProduct::TimProduct(diagnostic_updater::Updater * updater) : updater_(updater)
{
}

//
// U-Blox Time Sync Products, partially implemented.
//
void TimProduct::getRosParams(const rclcpp::Node * node, const GNSS & gnss) {
  (void)gnss;
  bool tim_enabled = node->get_parameter("publish.tim.all");
  bool rxm_enabled = node->get_parameter("publish.rxm.all");
  tim_tm2_enabled_ = node->declare_parameter("publish.tim.tm2", tim_enabled);
  rxm_sfrb_enabled_ = node->declare_parameter("publish.rxm.sfrb", rxm_enabled);
  rxm_raw_enabled_ = node->declare_parameter("publish.rxm.raw", rxm_enabled);
}

bool TimProduct::configureUblox(const GNSS & gnss) {
  (void)gnss;
  uint8_t r = 1;
  // Configure the reciever
  if (!gps.setUTCtime()) {
    throw std::runtime_error(std::string("Failed to Configure TIM Product to UTC Time"));
  }

  if (!gps.setTimtm2(r)) {
    throw std::runtime_error(std::string("Failed to Configure TIM Product"));
  }

  return true;
}

void TimProduct::subscribe(const rclcpp::Node * node) {
  // ROS_INFO("TIM is Enabled: %u", enabled["tim"]);
  // ROS_INFO("TIM-TM2 is Enabled: %u", enabled["tim_tm2"]);
  // Subscribe to TIM-TM2 messages (Time mark messages)

  gps.subscribe<ublox_msgs::msg::TimTM2>(std::bind(
    &TimProduct::callbackTimTM2, this, std::placeholders::_1), 1);

  // ROS_INFO("Subscribed to TIM-TM2 messages on topic tim/tm2");

  // Subscribe to SFRBX messages
  if (rxm_sfrb_enabled_) {
    gps.subscribe<ublox_msgs::msg::RxmSFRBX>(std::bind(
        publish<ublox_msgs::msg::RxmSFRBX>, std::placeholders_1, "rxmsfrb"), 1);
  }

  // Subscribe to RawX messages
  if (rxm_raw_enabled_) {
    gps.subscribe<ublox_msgs::msg::RxmRAWX>(std::bind(
        publish<ublox_msgs::msg::RxmRAWX>, std::placeholders::_1, "rxmraw"), 1);
  }

  if (tim_tm2_enabled_) {
    tim_tm2_pub_ = node->create_publisher<ublox_msgs::msg::TimTM2>("timtm2", 1);
    time_ref_pub_ = node->create_publisher<sensor_msgs::msg::TimeReference>("interrupt_time", 1);
  }
}

void TimProduct::callbackTimTM2(const ublox_msgs::msg::TimTM2 &m) {

  if (tim_tm2_enabled_) {
    auto t_ref = std::make_unique<sensor_msgs::msg::TimeReference>();
    // create time ref message and put in the data
    t_ref->header.stamp = rclcpp::Time::now();
    t_ref->header.frame_id = frame_id;

    t_ref->time_ref = rclcpp::Time((m.wnR * 604800 + m.towMsR / 1000), (m.towMsR % 1000) * 1000000 + m.towSubMsR);

    std::ostringstream src;
    src << "TIM" << int(m.ch);
    t_ref->source = src.str();

    tim_tm2_pub_->publish(m);
    time_ref_pub_->publish(std::move(t_ref));
  }

  updater_->force_update();
}

void TimProduct::initializeRosDiagnostics() {
  updater_->force_update();
}

}
