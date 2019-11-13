#ifndef UBLOX_GPS_UBLOX_FIRMWARE_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE_HPP

#include <memory>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace ublox_gps {

//! Topic diagnostics for fix / fix_velocity messages
struct FixDiagnostic {
  // Must not copy this struct (would confuse FrequencyStatusParam pointers)
  FixDiagnostic(const FixDiagnostic&) = delete;

  /**
   * @brief Add a topic diagnostic to the diagnostic updater for fix topics.
   *
   * @details The minimum and maximum frequency are equal to the nav rate in Hz.
   * @param name the ROS topic
   * @param freq_tol the tolerance [%] for the topic frequency
   * @param freq_window the number of messages to use for diagnostic statistics
   * @param stamp_min the minimum allowed time delay
   */
  explicit FixDiagnostic(diagnostic_updater::Updater * updater, const std::string & name, double freq_tol, int freq_window,
                         double stamp_min) {
    const double target_freq = 1.0 / (meas_rate * 1e-3 * nav_rate); // Hz
    min_freq = target_freq;
    max_freq = target_freq;
    diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq,
                                                        freq_tol, freq_window);
    double stamp_max = meas_rate * 1e-3 * (1 + freq_tol);
    diagnostic_updater::TimeStampStatusParam time_param(stamp_min, stamp_max);
    diagnostic = std::make_unique<diagnostic_updater::TopicDiagnostic>(name,
                                                         *updater,
                                                         freq_param,
                                                         time_param);
  }

  //! Topic frequency diagnostic updater
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> diagnostic;
  //! Minimum allow frequency of topic
  double min_freq;
  //! Maximum allow frequency of topic
  double max_freq;
};

/**
 * @brief This abstract class represents a firmware component.
 *
 * @details The Firmware components update the fix diagnostics.
 */
class UbloxFirmware : public virtual ComponentInterface {
 public:

  //! Tolerance for Fix topic frequency as percentage of target frequency
  constexpr static double kFixFreqTol = 0.15;
  //! Window [num messages] for Fix Frequency Diagnostic
  constexpr static double kFixFreqWindow = 10;
  //! Minimum Time Stamp Status for fix frequency diagnostic
  constexpr static double kTimeStampStatusMin = 0;

  explicit UbloxFirmware(diagnostic_updater::Updater * updater) : updater_(updater)
  {
    // configure diagnostic updater for frequency
    freq_diag_ = std::make_unique<FixDiagnostic>(updater_, std::string("fix"), kFixFreqTol,
                                                 kFixFreqWindow, kTimeStampStatusMin);
  }

  /**
   * @brief Add the fix diagnostics to the updater.
   */
  void initializeRosDiagnostics()
  {
  }

 protected:
  /**
   * @brief Handle to send fix status to ROS diagnostics.
   */
  virtual void fixDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper& stat) = 0;

  diagnostic_updater::Updater *updater_;
  //! fix frequency diagnostic updater
  std::unique_ptr<FixDiagnostic> freq_diag_;

  //! The fix status service type, set in the Firmware Component
  //! based on the enabled GNSS
  int fix_status_service_;
};

}  // namespace ublox_gps

#endif
