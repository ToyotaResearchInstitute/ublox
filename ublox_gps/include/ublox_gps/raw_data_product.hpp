#ifndef UBLOX_GPS_RAW_DATA_PRODUCT_HPP
#define UBLOX_GPS_RAW_DATA_PRODUCT_HPP

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>

namespace ublox_gps {

/**
 * @brief Implements functions for Raw Data products.
 */
class RawDataProduct: public virtual ComponentInterface {
 public:
  static constexpr double kRtcmFreqTol = 0.15;
  static constexpr int kRtcmFreqWindow = 25;

  /**
   * @brief Does nothing since there are no Raw Data product specific settings.
   */
  void getRosParams(const rclcpp::Node * node, const GNSS & gnss) override;

  /**
   * @brief Does nothing since there are no Raw Data product specific settings.
   * @return always returns true
   */
  bool configureUblox(const GNSS & gnss) override { (void)gnss; return true; }

  /**
   * @brief Adds frequency diagnostics for RTCM topics.
   */
  void initializeRosDiagnostics() override;

  /**
   * @brief Subscribe to Raw Data Product messages and set up ROS publishers.
   *
   * @details Subscribe to RxmALM, RxmEPH, RxmRAW, and RxmSFRB messages.
   */
  void subscribe(const rclcpp::Node * node) override;

 private:
  //! Topic diagnostic updaters
  std::vector<std::unique_ptr<UbloxTopicDiagnostic> > freq_diagnostics_;

  bool rxm_enabled;
  bool rxm_raw_enabled_;
  bool rxm_sfrb_enabled_;
  bool rxm_eph_enabled_;
  bool rxm_alm_enabled_;
};

}  // namespace ublox_gps

#endif
