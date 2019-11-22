#ifndef UBLOX_GPS_UBLOX_TOPIC_DIAGNOSTIC_HPP
#define UBLOX_GPS_UBLOX_TOPIC_DIAGNOSTIC_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/publisher.hpp>
#include <diagnostic_updater/update_functions.hpp>

namespace ublox_gps {

//! Topic diagnostics for u-blox messages
struct UbloxTopicDiagnostic {
  UbloxTopicDiagnostic() {}

  // Must not copy this struct (would confuse FrequencyStatusParam pointers)
  UbloxTopicDiagnostic(const UbloxTopicDiagnostic&) = delete;

  /**
   * @brief Add a topic diagnostic to the diagnostic updater for
   *
   * @details The minimum and maximum frequency are equal to the nav rate in Hz.
   * @param name the ROS topic
   * @param freq_tol the tolerance [%] for the topic frequency
   * @param freq_window the number of messages to use for diagnostic statistics
   */
  UbloxTopicDiagnostic(std::string topic, double freq_tol, int freq_window) {
    const double target_freq = 1.0 / (meas_rate * 1e-3 * nav_rate); // Hz
    min_freq = target_freq;
    max_freq = target_freq;
    diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq,
                                                        freq_tol, freq_window);
    diagnostic = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(topic,
                                                                   *updater,
                                                                   freq_param);
  }

  /**
   * @brief Add a topic diagnostic to the diagnostic updater for
   *
   * @details The minimum and maximum frequency are equal to the nav rate in Hz.
   * @param name the ROS topic
   * @param freq_min the minimum acceptable frequency for the topic
   * @param freq_max the maximum acceptable frequency for the topic
   * @param freq_tol the tolerance [%] for the topic frequency
   * @param freq_window the number of messages to use for diagnostic statistics
   */
  UbloxTopicDiagnostic(std::string topic, double freq_min, double freq_max,
                       double freq_tol, int freq_window) {
    min_freq = freq_min;
    max_freq = freq_max;
    diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq,
                                                        freq_tol, freq_window);
    diagnostic = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(topic,
                                                                   *updater,
                                                                   freq_param);
  }

  //! Topic frequency diagnostic updater
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> diagnostic;
  //! Minimum allow frequency of topic
  double min_freq;
  //! Maximum allow frequency of topic
  double max_freq;
};

}  // namespace ublox_gps

#endif
