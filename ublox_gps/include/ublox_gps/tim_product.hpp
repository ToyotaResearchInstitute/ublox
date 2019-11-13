#ifndef UBLOX_GPS_TIM_PRODUCT_HPP
#define UBLOX_GPS_TIM_PRODUCT_HPP

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/tim_tm2.hpp>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

/**
 * @brief Implements functions for Time Sync products.
 * @todo partially implemented
 */
class TimProduct: public virtual ComponentInterface {
  explicit TimProduct(diagnostic_updater::Updater * updater);

  /**
   * @brief Get the Time Sync parameters.
   * @todo Currently unimplemented.
   */
  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override;

  /**
   * @brief Configure Time Sync settings.
   * @todo Currently unimplemented.
   */
  bool configureUblox(const GNSS & gnss) override;

  /**
   * @brief Adds diagnostic updaters for Time Sync status.
   * @todo Currently unimplemented.
   */
  void initializeRosDiagnostics() override;

  /**
   * @brief Subscribe to Time Sync messages.
   *
   * @details Subscribes to RxmRAWX & RxmSFRBX messages.
   */
  void subscribe(const rclcpp::Node * node) override;

 protected:
  /**
   * @brief
   * @details Publish recieved TimTM2 messages if enabled
   */
  void callbackTimTM2(const ublox_msgs::msg::TimTM2 &m);

  rclcpp::Publisher<ublox_msgs::msg::TimTM2>::SharedPtr tim_tm2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;

  diagnostic_updater::Updater *updater_;

  bool tim_tm2_enabled_;
  bool rxm_sfrb_enabled_;
  bool rxm_raw_enabled_;
};

}  // namespace ublox_gps

#endif
