//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_UBLOX_NODE_HPP
#define UBLOX_GPS_UBLOX_NODE_HPP

// STL
#include <limit>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

// ROS includes
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

// Ublox GPS includes
#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/raw_data_pa.hpp>
#include <ublox_gps/utils.hpp>

// This file declares the ComponentInterface which acts as a high level
// interface for u-blox firmware, product categories, etc. It contains methods
// to configure the u-blox and subscribe to u-blox messages.
//
// This file also declares UbloxNode which implements ComponentInterface and is
// the main class and ros node. it implements functionality which applies to
// any u-blox device, regardless of the firmware version or product type.
// The class is designed in compositional style; it contains ComponentInterfaces
// which implement features specific to the device based on its firmware version
// and product category. UbloxNode calls the public methods of each component.
//
// This file declares UbloxFirmware is an abstract class which implements
// ComponentInterface and functions generic to all firmware (such as the
// initializing the fix diagnostics). Subclasses of UbloxFirmware for firmware
// versions 6-8 are also declared in this file.
//
// Lastly, this file declares classes for each product category which also
// implement u-blox interface, currently only the class for High Precision
// GNSS devices has been fully implemented and tested.

/**
 * @namespace ublox_gps
 * This namespace is for the ROS u-blox node and handles anything regarding
 * ROS parameters, message passing, diagnostics, etc.
 */
namespace ublox_gps {

//! The measurement [ms], see CfgRate.msg
uint16_t meas_rate;
//! IDs of RTCM out messages to configure.
std::vector<uint8_t> rtcm_ids;
//! Rates of RTCM out messages. Size must be the same as rtcm_ids
std::vector<uint8_t> rtcm_rates;

/**
 * @brief Check that the parameter is in the range.
 * @param val the value to check
 * @param min the minimum for this value
 * @param max the maximum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error if it is out of bounds
 */
template <typename V, typename T>
void checkRange(V val, T min, T max, std::string name) {
  if (val < min || val > max) {
    std::stringstream oss;
    oss << "Invalid settings: " << name << " must be in range [" << min <<
        ", " << max << "].";
    throw std::runtime_error(oss.str());
  }
}

/**
 * @brief Check that the elements of the vector are in the range.
 * @param val the vector to check
 * @param min the minimum for this value
 * @param max the maximum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error value it is out of bounds
 */
template <typename V, typename T>
void checkRange(std::vector<V> val, T min, T max, std::string name) {
  for (size_t i = 0; i < val.size(); i++)  {
    std::stringstream oss;
    oss << name << "[" << i << "]";
    checkRange(val[i], min, max, oss.str());
  }
}

/**
 * @brief Get a unsigned integer value from the parameter server.
 * @param key the key to be used in the parameter server's dictionary
 * @param u storage for the retrieved value.
 * @throws std::runtime_error if the parameter is out of bounds
 * @return true if found, false if not found.
 */
template <typename U>
bool getRosUint(const std::string& key, U &u) {
  int param;
  if (!nh->getParam(key, param)) {
    return false;
  }
  // Check the bounds
  U min = std::numeric_limits<U>::lowest();
  U max = std::numeric_limits<U>::max();
  checkRange(param, min, max, key);
  // set the output
  u = (U) param;
  return true;
}

/**
 * @brief Get a unsigned integer value from the parameter server.
 * @param key the key to be used in the parameter server's dictionary
 * @param u storage for the retrieved value.
 * @param val value to use if the server doesn't contain this parameter.
 * @throws std::runtime_error if the parameter is out of bounds
 * @return true if found, false if not found.
 */
template <typename U, typename V>
void getRosUint(const std::string& key, U &u, V default_val) {
  if (!getRosUint(key, u)) {
    u = default_val;
  }
}

/**
 * @brief Get a unsigned integer vector from the parameter server.
 * @throws std::runtime_error if the parameter is out of bounds.
 * @return true if found, false if not found.
 */
template <typename U>
bool getRosUint(const std::string& key, std::vector<U> &u) {
  std::vector<int> param;
  if (!nh->getParam(key, param)) {
    return false;
  }

  // Check the bounds
  U min = std::numeric_limits<U>::lowest();
  U max = std::numeric_limits<U>::max();
  checkRange(param, min, max, key);

  // set the output
  u.insert(u.begin(), param.begin(), param.end());
  return true;
}

/**
 * @brief Get a integer (size 8 or 16) value from the parameter server.
 * @param key the key to be used in the parameter server's dictionary
 * @param u storage for the retrieved value.
 * @throws std::runtime_error if the parameter is out of bounds
 * @return true if found, false if not found.
 */
template <typename I>
bool getRosInt(const std::string& key, I &u) {
  int param;
  if (!nh->getParam(key, param)) {
    return false;
  }
  // Check the bounds
  I min = std::numeric_limits<I>::lowest();
  I max = std::numeric_limits<I>::max();
  checkRange(param, min, max, key);
  // set the output
  u = (I) param;
  return true;
}

/**
 * @brief Get an integer value (size 8 or 16) from the parameter server.
 * @param key the key to be used in the parameter server's dictionary
 * @param u storage for the retrieved value.
 * @param val value to use if the server doesn't contain this parameter.
 * @throws std::runtime_error if the parameter is out of bounds
 * @return true if found, false if not found.
 */
template <typename U, typename V>
void getRosInt(const std::string& key, U &u, V default_val) {
  if (!getRosInt(key, u)) {
    u = default_val;
  }
}

/**
 * @brief Get a int (size 8 or 16) vector from the parameter server.
 * @throws std::runtime_error if the parameter is out of bounds.
 * @return true if found, false if not found.
 */
template <typename I>
bool getRosInt(const std::string& key, std::vector<I> &i) {
  std::vector<int> param;
  if (!nh->getParam(key, param)) {
    return false;
  }

  // Check the bounds
  I min = std::numeric_limits<I>::lowest();
  I max = std::numeric_limits<I>::max();
  checkRange(param, min, max, key);

  // set the output
  i.insert(i.begin(), param.begin(), param.end());
  return true;
}

/**
 * @brief Publish a ROS message of type MessageT.
 *
 * @details This function should be used to publish all messages which are
 * simply read from u-blox and published.
 * @param m the message to publish
 * @param topic the topic to publish the message on
 */
template <typename MessageT>
void publish(const MessageT& m, const std::string& topic) {
  static ros::Publisher publisher = nh->advertise<MessageT>(topic, 1);
  publisher.publish(m);
}

/**
 * @brief This class represents u-blox ROS node for *all* firmware and product
 * versions.
 *
 * @details It loads the user parameters, configures the u-blox
 * device, subscribes to u-blox messages, and configures the device hardware.
 * Functionality specific to a given product or firmware version, etc. should
 * NOT be implemented in this class. Instead, the user should add the
 * functionality to the appropriate implementation of ComponentInterface.
 * If necessary, the user should create a class which implements u-blox
 * interface, then add a pointer to an instance of the class to the
 * components vector.
 * The UbloxNode calls the public methods of ComponentInterface for each
 * element in the components vector.
 */
class UbloxNode final : public rclcpp::Node {
 public:
  //! How long to wait during I/O reset [s]
  constexpr static int kResetWait = 10;
  //! how often (in seconds) to call poll messages
  constexpr static double kPollDuration = 1.0;
  // Constants used for diagnostic frequency updater
  //! [s] 5Hz diagnostic period
  constexpr static float kDiagnosticPeriod = 0.2;

  /**
   * @brief Initialize and run the u-blox node.
   */
  explicit UbloxNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Get the node parameters from the ROS Parameter Server.
   */
  void getRosParams();

  /**
   * @brief Configure the device based on ROS parameters.
   * @return true if configured successfully
   */
  bool configureUblox();

  /**
   * @brief Subscribe to all requested u-blox messages.
   */
  void subscribe();

  /**
   * @brief Initialize the diagnostic updater and add the fix diagnostic.
   */
  void initializeRosDiagnostics();

  /**
   * @brief Print an INF message to the ROS console.
   */
  void printInf(const ublox_msgs::msg::Inf &m, uint8_t id);

 private:

  /**
   * @brief Initialize the I/O handling.
   */
  void initializeIo();

  /**
   * @brief Initialize the U-Blox node. Configure the U-Blox and subscribe to
   * messages.
   */
  void initialize();

  /**
   * @brief Shutdown the node. Closes the serial port.
   */
  void shutdown();

  /**
   * @brief Send a reset message the u-blox device & re-initialize the I/O.
   * @return true if reset was successful, false otherwise.
   */
  bool resetDevice();

  /**
   * @brief Process the MonVer message and add firmware and product components.
   *
   * @details Determines the protocol version, product type and supported GNSS.
   */
  void processMonVer();

  /**
   * @brief Add the interface for firmware specific configuration, subscribers,
   * & diagnostics. This assumes the protocol_version_ has been set.
   */
  void addFirmwareInterface();

  /**
   * @brief Add the interface which is used for product category
   * configuration, subscribers, & diagnostics.
   * @param the product category, i.e. SPG, HPG, ADR, UDR, TIM, or FTS.
   * @param for HPG/TIM products, this value is either REF or ROV, for other
   * products this string is empty
   */
  void addProductInterface(std::string product_category,
                           std::string ref_rov = "");

  /**
   * @brief Poll messages from the U-Blox device.
   * @param event a timer indicating how often to poll the messages
   */
  void pollMessages(const ros::TimerEvent& event);

  /**
   * @brief Configure INF messages, call after subscribe.
   */
  void configureInf();

  std::unique_ptr<diagnostic_updater::Updater> updater_;

  //! The u-blox node components
  /*!
   * The node will call the functions in these interfaces for each object
   * in the vector.
   */
  std::vector<std::unique_ptr<ComponentInterface> > components_;

  //! Determined From Mon VER
  float protocol_version_ = 0;
  // Variables set from parameter server
  //! Device port
  std::string device_;
  //! dynamic model type
  std::string dynamic_model_;
  //! Fix mode type
  std::string fix_mode_;
  //! Set from dynamic model string
  uint8_t dmodel_;
  //! Set from fix mode string
  uint8_t fmode_;
  //! UART1 baudrate
  uint32_t baudrate_;
  //! UART in protocol (see CfgPRT message for constants)
  uint16_t uart_in_;
  //! UART out protocol (see CfgPRT message for constants)
  uint16_t uart_out_;
  //! USB TX Ready Pin configuration (see CfgPRT message for constants)
  uint16_t usb_tx_;
  //! Whether to configure the USB port
  /*! Set to true if usb_in & usb_out parameters are set */
  bool set_usb_;
  //! USB in protocol (see CfgPRT message for constants)
  uint16_t usb_in_;
  //! USB out protocol (see CfgPRT message for constants)
  uint16_t usb_out_ ;
  //! The measurement rate in Hz
  double rate_;
  //! If true, set configure the User-Defined Datum
  bool set_dat_;
  //! User-defined Datum
  ublox_msgs::msg::CfgDAT cfg_dat_;
  //! Whether or not to enable SBAS
  bool enable_sbas_;
  //! Whether or not to enable PPP (advanced setting)
  bool enable_ppp_;
  //! SBAS Usage parameter (see CfgSBAS message)
  uint8_t sbas_usage_;
  //! Max SBAS parameter (see CfgSBAS message)
  uint8_t max_sbas_;
  //! Dead reckoning limit parameter
  uint8_t dr_limit_;
  //! Parameters to load from non-volatile memory during configuration
  ublox_msgs::msg::CfgCFG load_;
  //! Parameters to save to non-volatile memory after configuration
  ublox_msgs::msg::CfgCFG save_;
  //! rate for TIM-TM2
  uint8_t tim_rate_;

  //! raw data stream logging
  RawDataStreamPa rawDataStreamPa_;

  GNSS gnss_;

  //! The ROS frame ID of this device
  std::string frame_id_;

  //! Navigation rate in measurement cycles, see CfgRate.msg
  uint16_t nav_rate_;

  bool all_enable_;
  bool inf_enable_;
  bool nav_enable_;
  bool rxm_enable_;
  bool aid_enable_;
  bool mon_enable_;

  bool nav_status_enable_;
  bool nav_posecef_enable_;
  bool nav_clock_enable_;

  bool inf_debug_enable_;
  bool inf_error_enable_;
  bool inf_notice_enable_;
  bool inf_test_enable_;
  bool inf_warning_enable_;

  bool aid_alm_enable_;
  bool aid_eph_enable_;
  bool aid_hui_enable_;

  float diagnostic_period_;

  bool raw_data_enable_;

  //! Handles communication with the U-Blox Device
  ublox_gps::Gps gps_;

  //! Flag for enabling configuration on startup
  bool config_on_startup_flag_;
};

}  // namespace ublox_gps

#endif
