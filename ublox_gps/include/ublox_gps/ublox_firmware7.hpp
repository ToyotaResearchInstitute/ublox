#ifndef UBLOX_GPS_UBLOX_FIRMWARE7_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE7_HPP

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_nmea7.hpp>
#include <ublox_msgs/msg/nav_pvt7.hpp>

#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware7plus.hpp>

namespace ublox_gps {

/**
 * @brief Implements functions for firmware version 7.
 */
class UbloxFirmware7 : public UbloxFirmware7Plus<ublox_msgs::msg::NavPVT7> {
 public:
  explicit UbloxFirmware7(diagnostic_updater::Updater * updater);

  /**
   * @brief Get the parameters specific to firmware version 7.
   *
   * @details Get the GNSS and NMEA settings.
   */
  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override;

  /**
   * @brief Configure GNSS individually. Only configures GLONASS.
   */
  bool configureUblox(const GNSS & gnss) override;

  /**
   * @brief Subscribe to messages which are not generic to all firmware.
   *
   * @details Subscribe to NavPVT7 messages, RxmRAW, and RxmSFRB messages.
   */
  void subscribe(const rclcpp::Node * node) override;

 private:
  //! Used to configure NMEA (if set_nmea_)
  /*!
   * Filled from ROS parameters
   */
  ublox_msgs::msg::CfgNMEA7 cfg_nmea_;
  //! Whether or not to Configure the NMEA settings
  bool set_nmea_;

  int fix_status_service_;
};

}  // namespace ublox_gps

#endif
