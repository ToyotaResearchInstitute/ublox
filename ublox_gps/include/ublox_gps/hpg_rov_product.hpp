#ifndef UBLOX_GPS_HPG_ROV_PRODUCT_HPP
#define UBLOX_GPS_HPG_ROV_PRODUCT_HPP

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/nav_rel_pos_ned.hpp>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>

namespace ublox_gps {

/**
 * @brief Implements functions for High Precision GNSS Rover devices.
 */
class HpgRovProduct: public virtual ComponentInterface {
 public:
  // Constants for diagnostic updater
  //! Diagnostic updater: RTCM topic frequency min [Hz]
  constexpr static double kRtcmFreqMin = 1;
  //! Diagnostic updater: RTCM topic frequency max [Hz]
  constexpr static double kRtcmFreqMax = 10;
  //! Diagnostic updater: RTCM topic frequency tolerance [%]
  constexpr static double kRtcmFreqTol = 0.1;
  //! Diagnostic updater: RTCM topic frequency window [num messages]
  constexpr static int kRtcmFreqWindow = 25;
  /**
   * @brief Get the ROS parameters specific to the Rover configuration.
   *
   * @details Get the DGNSS mode.
   */
  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override;

  /**
   * @brief Configure rover settings.
   *
   * @details Configure the DGNSS mode.
   * @return true if configured correctly, false otherwise
   */
  bool configureUblox(const GNSS & gnss) override;

  /**
   * @brief Add diagnostic updaters for rover GNSS status, including
   * status of RTCM messages.
   */
  void initializeRosDiagnostics() override;

  /**
   * @brief Subscribe to Rover messages, such as NavRELPOSNED.
   */
  void subscribe(const rclcpp::Node * node) override;

 protected:
  /**
   * @brief Update the rover diagnostics, including the carrier phase solution
   * status (float or fixed).
   */
  void carrierPhaseDiagnostics(
      diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Set the last received message and call rover diagnostic updater
   *
   * @details Publish received NavRELPOSNED messages if enabled
   */
  void callbackNavRelPosNed(const ublox_msgs::msg::NavRELPOSNED &m);


  //! Last relative position (used for diagnostic updater)
  ublox_msgs::msg::NavRELPOSNED last_rel_pos_;

  //! The DGNSS mode
  /*! see CfgDGNSS message for possible values */
  uint8_t dgnss_mode_;

  bool nav_relposned_enabled_;

  rclcpp::Publisher<ublox_msgs::msg::NavRELPOSNED>::SharedPtr publisher_;

  //! The RTCM topic frequency diagnostic updater
  UbloxTopicDiagnostic freq_rtcm_;
};

}  // namespace ublox_gps

#endif
