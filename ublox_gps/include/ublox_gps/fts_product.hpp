#ifndef UBLOX_GPS_FTS_PRODUCT_HPP
#define UBLOX_GPS_FTS_PRODUCT_HPP

#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

/**
 * @brief Implements functions for FTS products. Currently unimplemented.
 * @todo Unimplemented.
 */
class FtsProduct: public virtual ComponentInterface {
  /**
   * @brief Get the FTS parameters.
   * @todo Currently unimplemented.
   */
  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override {
    (void)node;
    (void)gnss;
    // ROS_WARN("Functionality specific to u-blox FTS devices is %s",
    //          "unimplemented. See FtsProduct class in node.h & node.cpp.");
  }

  /**
   * @brief Configure FTS settings.
   * @todo Currently unimplemented.
   */
  bool configureUblox(const GNSS & gnss) override {
    (void)gnss;
    return false;
  }

  /**
   * @brief Adds diagnostic updaters for FTS status.
   * @todo Currently unimplemented.
   */
  void initializeRosDiagnostics() override {}

  /**
   * @brief Subscribe to FTS messages.
   * @todo Currently unimplemented.
   */
  void subscribe(const rclcpp::Node * node) override {(void)node;}
};

}  // namespace ublox_gps

#endif
