#ifndef UBLOX_GPS_UBLOX_FIRMWARE7PLUS_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE7PLUS_HPP

#include <cmath>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <ublox_msgs/msg/nav_pvt.hpp>

namespace ublox_gps {

/**
 * @brief Abstract class for Firmware versions >= 7.
 *
 * @details This class keeps track of the last NavPVT message uses it to
 * update the fix diagnostics. It is a template class because the NavPVT message
 * is a different length for firmware versions 7 and 8.
 *
 * @typedef NavPVT the NavPVT message type for the given firmware version
 */
template<typename NavPVT>
class UbloxFirmware7Plus : public UbloxFirmware {
 public:

  // TODO(clalancette): this probably can't work
  explicit UbloxFirmware7Plus(const rclcpp::Node * node, diagnostic_updater::Updater updater) : UbloxFirmware(updater) {
    navpvt_pub_ = node->create_publisher<ublox_msgs::msg::NavPVT>("navpvt", 1);
    fix_pub_ = node->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);
    velocity_pub_ = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped("fix_velocity", 1);
  }

  /**
   * @brief Publish a NavSatFix and TwistWithCovarianceStamped messages.
   *
   * @details If a fixed carrier phase solution is available, the NavSatFix
   * status is set to GBAS fixed. If NavPVT publishing is enabled, the message
   * is published. This function also calls the ROS diagnostics updater.
   * @param m the message to publish
   */
  void callbackNavPvt(const ublox_msgs::msg::NavPVT& m) {
    if (enabled["nav_pvt"]) {
      // NavPVT publisher
      navpvt_pub_->publish(m);
    }

    //
    // NavSatFix message
    //
    sensor_msgs::msg::NavSatFix fix;
    fix.header.frame_id = frame_id;
    // set the timestamp
    uint8_t valid_time = m.VALID_DATE | m.VALID_TIME | m.VALID_FULLY_RESOLVED;
    if (((m.valid & valid_time) == valid_time) &&
        (m.flags2 & m.FLAGS2_CONFIRMED_AVAILABLE)) {
      // Use NavPVT timestamp since it is valid
      // The time in nanoseconds from the NavPVT message can be between -1e9 and 1e9
      //  The ros time uses only unsigned values, so a negative nano seconds must be
      //  converted to a positive value
      if (m.nano < 0) {
        fix.header.stamp.sec = toUtcSeconds(m) - 1;
        fix.header.stamp.nsec = (uint32_t)(m.nano + 1e9);
      }
      else {
        fix.header.stamp.sec = toUtcSeconds(m);
        fix.header.stamp.nsec = (uint32_t)(m.nano);
      }
    } else {
      // Use ROS time since NavPVT timestamp is not valid
      fix.header.stamp = rclcpp::Time::now();
    }
    // Set the LLA
    fix.latitude = m.lat * 1e-7; // to deg
    fix.longitude = m.lon * 1e-7; // to deg
    fix.altitude = m.height * 1e-3; // to [m]
    // Set the Fix status
    bool fixOk = m.flags & m.FLAGS_GNSS_FIX_OK;
    if (fixOk && m.fixType >= m.FIX_TYPE_2D) {
      fix.status.status = fix.status.STATUS_FIX;
      if (m.flags & m.CARRIER_PHASE_FIXED) {
        fix.status.status = fix.status.STATUS_GBAS_FIX;
      }
    } else {
      fix.status.status = fix.status.STATUS_NO_FIX;
    }
    // Set the service based on GNSS configuration
    fix.status.service = fix_status_service_;

    // Set the position covariance
    const double varH = pow(m.hAcc / 1000.0, 2); // to [m^2]
    const double varV = pow(m.vAcc / 1000.0, 2); // to [m^2]
    fix.position_covariance[0] = varH;
    fix.position_covariance[4] = varH;
    fix.position_covariance[8] = varV;
    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    fix_pub_->publish(fix);

    //
    // Twist message
    //
    geometry_msgs::msg::TwistWithCovarianceStamped velocity;
    velocity.header.stamp = fix.header.stamp;
    velocity.header.frame_id = frame_id;

    // convert to XYZ linear velocity [m/s] in ENU
    velocity.twist.twist.linear.x = m.vel_e * 1e-3;
    velocity.twist.twist.linear.y = m.vel_n * 1e-3;
    velocity.twist.twist.linear.z = -m.vel_d * 1e-3;
    // Set the covariance
    const double covSpeed = pow(m.s_acc * 1e-3, 2);
    const int cols = 6;
    velocity.twist.covariance[cols * 0 + 0] = covSpeed;
    velocity.twist.covariance[cols * 1 + 1] = covSpeed;
    velocity.twist.covariance[cols * 2 + 2] = covSpeed;
    velocity.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

    velocity_pub_->publish(velocity);

    //
    // Update diagnostics
    //
    last_nav_pvt_ = m;
    freq_diag_->diagnostic->tick(fix.header.stamp);
    updater_->update();
  }

 protected:

  /**
   * @brief Update the fix diagnostics from Nav PVT message.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    // check the last message, convert to diagnostic
    if (last_nav_pvt_.fix_type ==
        ublox_msgs::msg::NavPVT::FIX_TYPE_DEAD_RECKONING_ONLY) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "Dead reckoning only";
    } else if (last_nav_pvt_.fix_type == ublox_msgs::msg::NavPVT::FIX_TYPE_2D) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "2D fix";
    } else if (last_nav_pvt_.fix_type == ublox_msgs::msg::NavPVT::FIX_TYPE_3D) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "3D fix";
    } else if (last_nav_pvt_.fix_type ==
               ublox_msgs::msg::NavPVT::FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "GPS and dead reckoning combined";
    } else if (last_nav_pvt_.fix_type ==
               ublox_msgs::msg::NavPVT::FIX_TYPE_TIME_ONLY) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "Time only fix";
    }

    // If fix not ok (w/in DOP & Accuracy Masks), raise the diagnostic level
    if (!(last_nav_pvt_.flags & ublox_msgs::msg::NavPVT::FLAGS_GNSS_FIX_OK)) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message += ", fix not ok";
    }
    // Raise diagnostic level to error if no fix
    if (last_nav_pvt_.fix_type == ublox_msgs::msg::NavPVT::FIX_TYPE_NO_FIX) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      stat.message = "No fix";
    }

    // append last fix position
    stat.add("iTOW [ms]", last_nav_pvt_.i_tow);
    stat.add("Latitude [deg]", last_nav_pvt_.lat * 1e-7);
    stat.add("Longitude [deg]", last_nav_pvt_.lon * 1e-7);
    stat.add("Altitude [m]", last_nav_pvt_.height * 1e-3);
    stat.add("Height above MSL [m]", last_nav_pvt_.h_msl * 1e-3);
    stat.add("Horizontal Accuracy [m]", last_nav_pvt_.h_acc * 1e-3);
    stat.add("Vertical Accuracy [m]", last_nav_pvt_.v_acc * 1e-3);
    stat.add("# SVs used", last_nav_pvt_.num_sv);
  }

  //! The last received NavPVT message
  ublox_msgs::msg::NavPVT last_nav_pvt_;
  // Whether or not to enable the given GNSS
  //! Whether or not to enable GPS
  bool enable_gps_;
  //! Whether or not to enable GLONASS
  bool enable_glonass_;
  //! Whether or not to enable QZSS
  bool enable_qzss_;
  //! Whether or not to enable SBAS
  bool enable_sbas_;
  //! The QZSS Signal configuration, see CfgGNSS message
  uint32_t qzss_sig_cfg_;

  rclcpp::Publisher<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
};

}  // namespace ublox_gps

#endif
