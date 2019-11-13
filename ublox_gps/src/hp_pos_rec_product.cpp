#include <cmath>
#include <functional>
#include <memory>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <ublox_msgs/msg/nav_rel_pos_ned9.hpp>

namespace ublox_gps {

//
// U-Blox High Precision Positioning Receiver
//

HpPosRecProduct::HpPosRecProduct(diagnostic_updater::Updater * updater) : updater_(updater)
{
}

void HpPosRecProduct::getRosParams(const rclcpp::Node * node, const GNSS & gnss) {
  // Whether to publish Nav Relative Position NED
  bool nav_enabled = node->get_parameter("publish.nav.all");
  nav_relposned_enabled_ = node->declare_parameter("publish.nav.relposned", nav_enabled);
  .. Whether to publish the Heading info from Nav Relative Position NED
  nav_hading_enabled_ = node->declare_parameter("publish.nav.heading", nav_enabled);
}

void HpPosRecProduct::subscribe(const rclcpp::Node * node) {
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  gps.subscribe<ublox_msgs::msg::NavRELPOSNED9>(std::bind(
     &HpPosRecProduct::callbackNavRelPosNed, this, std::placeholders::_1), 1);

  if (nav_relposned_enabled_) {
    navrelposned_pub_ = node->create_publisher<ublox_msgs::msg::NavRELPOSNED9>("navrelposned", 1);
  }

  if (nav_heading_enabled_) {
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("navheading", 1);
  }
}

void HpPosRecProduct::callbackNavRelPosNed(const ublox_msgs::msg::NavRELPOSNED9 &m) {
  if (nav_relposned_enabled_) {
    navrelposned_pub_->publish(m);
  }
  if (nav_heading_enabled_) {
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

    imu_msg->header.stamp = rclcpp::Time::now();
    imu_msg->header.frame_id = frame_id;

    imu_msg->linear_acceleration_covariance[0] = -1;
    imu_msg->angular_velocity_covariance[0] = -1;

    double heading = static_cast<double>(m.relPosHeading) * 1e-5 / 180.0 * M_PI;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, heading);
    imu_msg->orientation.x = orientation[0];
    imu_msg->orientation.y = orientation[1];
    imu_msg->orientation.z = orientation[2];
    imu_msg->orientation.w = orientation[3];
    // Only heading is reported with an accuracy in 0.1mm units
    imu_msg->orientation_covariance[0] = 1000.0;
    imu_msg->orientation_covariance[4] = 1000.0;
    imu_msg->orientation_covariance[8] = pow(m.accHeading / 10000.0, 2);

    imu_pub_->publish(std::move(imu_msg));
  }

  last_rel_pos_ = m;
  updater_->update();
}

}
