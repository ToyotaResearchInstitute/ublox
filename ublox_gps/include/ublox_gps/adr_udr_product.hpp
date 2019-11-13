#ifndef UBLOX_GPS_ADR_UDR_PRODUCT_HPP
#define UBLOX_GPS_ADR_UDR_PRODUCT_HPP

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/esf_meas.hpp>
#include <ublox_msgs/msg/tim_tm2.hpp>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

/**
 * @brief Implements functions for Automotive Dead Reckoning (ADR) and
 * Untethered Dead Reckoning (UDR) Devices.
 */
class AdrUdrProduct: public virtual ComponentInterface {
 public:
  /**
   * @brief Get the ADR/UDR parameters.
   *
   * @details Get the use_adr parameter and check that the nav_rate is 1 Hz.
   */
  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override;

  /**
   * @brief Configure ADR/UDR settings.
   * @details Configure the use_adr setting.
   * @return true if configured correctly, false otherwise
   */
  bool configureUblox(const GNSS & gnss) override;

  /**
   * @brief Initialize the ROS diagnostics for the ADR/UDR device.
   * @todo unimplemented
   */
  void initializeRosDiagnostics() override {
    // ROS_WARN("ROS Diagnostics specific to u-blox ADR/UDR devices is %s",
    //          "unimplemented. See AdrUdrProduct class in node.h & node.cpp.");
  }

  /**
   * @brief Subscribe to ADR/UDR messages.
   *
   * @details Subscribe to NavATT, ESF and HNR messages based on user
   * parameters.
   */
  void subscribe(const rclcpp::Node * node) override;

 protected:
  //! Whether or not to enable dead reckoning
  bool use_adr_;

  bool esf_enabled_;
  bool nav_att_enabled_;
  bool esf_ins_enabled_;
  bool esf_meas_enabled_;
  bool esf_raw_enabled_;
  bool esf_status_enabled_;
  bool hnr_pvt_enabled_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;
  ublox_msgs::msg::TimTM2 timtm2;

  diagnostic_updater::Updater *updater_;

  void callbackEsfMEAS(const ublox_msgs::msg::EsfMEAS &m);
};

}  // namespace ublox_gps

#endif
