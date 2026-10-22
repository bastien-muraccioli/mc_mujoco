#pragma once

#include <algorithm>
#include <cmath>

namespace mc_mujoco
{

/**
 * @brief Digital 2nd-order (biquad) Butterworth high-pass filter.
 *
 * This is a standard IIR biquad designed via the bilinear transform of the analog
 * 2nd-order Butterworth high-pass prototype (maximally flat magnitude response,
 * i.e. quality factor Q = 1/sqrt(2)).
 *
 * It is meant to be applied independently on each scalar channel of a signal
 * (e.g. one instance per joint). Typical use here: extract the high-frequency
 * content of the estimated external/contact joint torque (qfrc_constraint +
 * qfrc_passive) so that only fast transients (e.g. the impact spike at
 * heel-strike while walking) go through, while the slowly varying/steady-state
 * part of the external torque (e.g. constant contact load) is rejected.
 *
 * Reference: Robert Bristow-Johnson's "Cookbook formulae for audio EQ biquad
 * filter coefficients", high-pass section, with Q fixed to the Butterworth
 * value.
 */
class ButterworthHP2
{
public:
  ButterworthHP2() = default;

  /**
   * @brief (Re-)compute the filter coefficients
   * @param cutoff_hz cutoff (-3dB) frequency of the high-pass filter [Hz]
   * @param sample_hz sampling frequency, i.e 1/dt, at which \ref update will be called [Hz]
   */
  void configure(double cutoff_hz, double sample_hz)
  {
    if(sample_hz <= 0.0)
    {
      return;
    }
    // Nyquist-safe clamping of the cutoff frequency
    cutoff_hz_ = std::clamp(cutoff_hz, 1e-3, 0.49 * sample_hz);
    sample_hz_ = sample_hz;

    constexpr double Q = 0.70710678118654752440; // 1/sqrt(2): maximally-flat (Butterworth) response

    const double w0 = 2.0 * M_PI * cutoff_hz_ / sample_hz_;
    const double cosw0 = std::cos(w0);
    const double sinw0 = std::sin(w0);
    const double alpha = sinw0 / (2.0 * Q);

    const double b0 = (1.0 + cosw0) / 2.0;
    const double b1 = -(1.0 + cosw0);
    const double b2 = (1.0 + cosw0) / 2.0;
    const double a0 = 1.0 + alpha;
    const double a1 = -2.0 * cosw0;
    const double a2 = 1.0 - alpha;

    b0_ = b0 / a0;
    b1_ = b1 / a0;
    b2_ = b2 / a0;
    a1_ = a1 / a0;
    a2_ = a2 / a0;
    configured_ = true;
  }

  /** @brief Reset the internal filter state (input/output history) to zero */
  void reset()
  {
    x1_ = x2_ = 0.0;
    y1_ = y2_ = 0.0;
  }

  /** @brief Feed one new sample and return the filtered output (direct form I difference equation) */
  double update(double x0)
  {
    if(!configured_)
    {
      return 0.0;
    }
    const double y0 = b0_ * x0 + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_;
    x1_ = x0;
    y2_ = y1_;
    y1_ = y0;
    return y0;
  }

  double cutoff() const noexcept
  {
    return cutoff_hz_;
  }

  double sampleRate() const noexcept
  {
    return sample_hz_;
  }

private:
  bool configured_ = false;
  double cutoff_hz_ = 0.0;
  double sample_hz_ = 0.0;
  double b0_ = 0.0, b1_ = 0.0, b2_ = 0.0;
  double a1_ = 0.0, a2_ = 0.0;
  double x1_ = 0.0, x2_ = 0.0;
  double y1_ = 0.0, y2_ = 0.0;
};

} // namespace mc_mujoco
