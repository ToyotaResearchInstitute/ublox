#ifndef UBLOX_GPS_UBLOX_FIRMWARE6_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE6_HPP

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <ublox_msgs/msg/cfg_nmea6.hpp>
#include <ublox_msgs/msg/nav_pos_llh.hpp>
#include <ublox_msgs/msg/nav_sol.hpp>
#include <ublox_msgs/msg/nav_vel_ned.hpp>

#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware.hpp>

namespace ublox_gps {

/**
 * @brief Implements functions for firmware version 6.
 */
class UbloxFirmware6 : public UbloxFirmware {
 public:
  explicit UbloxFirmware6(diagnostic_updater::Updater * updater);

  /**
   * @brief Sets the fix status service type to GPS.
   */
  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override;

  /**
   * @brief Prints a warning, GNSS configuration not available in this version.
   * @return true if configured correctly, false otherwise
   */
  bool configureUblox(const GNSS & gnss) override;

  /**
   * @brief Subscribe to NavPVT, RxmRAW, and RxmSFRB messages.
   */
  void subscribe(const rclcpp::Node * node) override;

 protected:
  /**
   * @brief Updates fix diagnostic from NavPOSLLH, NavVELNED, and NavSOL
   * messages.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

 private:
  /**
   * @brief Publish the fix and call the fix diagnostic updater.
   *
   * @details Also updates the last known position and publishes the NavPosLLH
   * message if publishing is enabled.
   * @param m the message to process
   */
  void callbackNavPosLlh(const ublox_msgs::msg::NavPOSLLH& m);

  /**
   * @brief Update the last known velocity.
   *
   * @details Publish the message if publishing is enabled.
   * @param m the message to process
   */
  void callbackNavVelNed(const ublox_msgs::msg::NavVELNED& m);

  /**
   * @brief Update the number of SVs used for the fix.
   *
   * @details Publish the message if publishing is enabled.
   * @param m the message to process
   */
  void callbackNavSol(const ublox_msgs::msg::NavSOL& m);

  //! The last received navigation position
  ublox_msgs::msg::NavPOSLLH last_nav_pos_;
  //! The last received navigation velocity
  ublox_msgs::msg::NavVELNED last_nav_vel_;
  //! The last received num SVs used
  ublox_msgs::msg::NavSOL last_nav_sol_;
  //! The last NavSatFix based on last_nav_pos_
  sensor_msgs::msg::NavSatFix fix_;
  //! The last Twist based on last_nav_vel_
  geometry_msgs::msg::TwistWithCovarianceStamped velocity_;

  //! Used to configure NMEA (if set_nmea_) filled with ROS parameters
  ublox_msgs::msg::CfgNMEA6 cfg_nmea_;
  //! Whether or not to configure the NMEA settings
  bool set_nmea_;

  bool nav_pos_llh_enabled_;
  bool nav_sol_enabled_;
  bool nav_velned_enabled_;
  bool nav_svinfo_enabled_;
  bool mon_hw_enabled_;

  rclcpp::Publisher<ublox_msgs::msg::NavPOSLLH>::SharedPtr nav_pos_llh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<ublox_msgs::msg::NavVELNED>::SharedPtr vel_ned_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<ublox_msgs::msg::NavSOL>::SharedPtr nav_sol_pub_;
};

}  // namespace ublox_gps

#endif
