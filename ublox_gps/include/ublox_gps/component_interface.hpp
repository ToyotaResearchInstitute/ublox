#ifndef UBLOX_GPS_COMPONENT_INTERFACE_HPP
#define UBLOX_GPS_COMPONENT_INTERFACE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

/**
 * @brief This interface is used to add functionality to the main node.
 *
 * @details This interface is generic and can be implemented for other features
 * besides the main node, hardware versions, and firmware versions.
 */
class ComponentInterface {
 public:
  /**
   * @brief Get the ROS parameters.
   * @throws std::runtime_error if a parameter is invalid or required
   * parameters are not set.
   */
  virtual void getRosParams(const rclcpp::Node * node, const GNSS & gnss) = 0;

  /**
   * @brief Configure the U-Blox settings.
   * @return true if configured correctly, false otherwise
   */
  virtual bool configureUblox(const GNSS & gnss) = 0;

  /**
   * @brief Initialize the diagnostics.
   *
   * @details Function may be empty.
   */
  virtual void initializeRosDiagnostics() = 0;

  /**
   * @brief Subscribe to u-blox messages and publish to ROS topics.
   */
  virtual void subscribe(const rclcpp::Node * node) = 0;
};

typedef std::shared_ptr<ComponentInterface> ComponentPtr;

}  // namespace ublox_gps

#endif
