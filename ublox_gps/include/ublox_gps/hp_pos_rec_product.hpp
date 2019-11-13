#ifndef UBLOX_GPS_HP_POS_REC_PRODUCT_HPP
#define UBLOX_GPS_HP_POS_REC_PRODUCT_HPP

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <ublox_msgs/msg/nav_rel_pos_ned9.hpp>

#include <ublox_gps/gnss.hpp>
#include <ublox_gps/hpg_ref_product.hpp>

namespace ublox_gps {

class HpPosRecProduct: public virtual HpgRefProduct {
 public:

  explicit HpPosRecProduct(diagnostic_updater::Updater * updater);

  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override;

  /**
   * @brief Subscribe to Rover messages, such as NavRELPOSNED.
   */
  void subscribe(const rclcpp::Node * node) override;

 protected:

  /**
   * @brief Set the last received message and call rover diagnostic updater
   *
   * @details Publish received NavRELPOSNED messages if enabled
   */
  void callbackNavRelPosNed(const ublox_msgs::msg::NavRELPOSNED9 &m);

  sensor_msgs::msg::Imu imu_;

  //! Last relative position (used for diagnostic updater)
  ublox_msgs::msg::NavRELPOSNED9 last_rel_pos_;

  bool nav_relposned_enabled_;
  bool nav_heading_enabled_;

  rclcpp::Publisher<ublox_msgs::msg::NavRELPOSNED9>::SharedPtr navrelposned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  diagnostic_updater::Updater * updater_;
};

}  // namespace ublox_gps

#endif
