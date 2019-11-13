#ifndef UBLOX_GPS_UBLOX_FIRMWARE8_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE8_HPP

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_nmea.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware7plus.hpp>

namespace ublox_gps {

/**
 *  @brief Implements functions for firmware version 8.
 */
class UbloxFirmware8 : public UbloxFirmware7Plus<ublox_msgs::msg::NavPVT> {
 public:
  explicit UbloxFirmware8(diagnostic_updater::Updater * updater);

  /**
   * @brief Get the ROS parameters specific to firmware version 8.
   *
   * @details Get the GNSS, NMEA, and UPD settings.
   */
  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override;

  /**
   * @brief Configure settings specific to firmware 8 based on ROS parameters.
   *
   * @details Configure GNSS, if it is different from current settings.
   * Configure the NMEA if desired by the user. It also may clear the
   * flash memory based on the ROS parameters.
   */
  bool configureUblox(const GNSS & gnss) override;

  /**
   * @brief Subscribe to u-blox messages which are not generic to all firmware
   * versions.
   *
   * @details Subscribe to NavPVT, NavSAT, MonHW, and RxmRTCM messages based
   * on user settings.
   */
  void subscribe(const rclcpp::Node * node) override;

 private:
  // Set from ROS parameters
  //! Whether or not to enable the Galileo GNSS
  bool enable_galileo_;
  //! Whether or not to enable the BeiDuo GNSS
  bool enable_beidou_;
  //! Whether or not to enable the IMES GNSS
  bool enable_imes_;
  //! Whether or not to configure the NMEA settings
  bool set_nmea_;
  //! Desired NMEA configuration.
  ublox_msgs::msg::CfgNMEA cfg_nmea_;
  //! Whether to clear the flash memory during configuration
  bool clear_bbr_;
  bool nav_pvt_enabled_;
  bool nav_sat_enabled_;
  bool mon_hw_enabled_;
  bool rxm_rtcm_enabled_;
};

}  // namespace ublox_gps

#endif
