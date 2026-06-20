#pragma once

#include <optional>
#include <string>

namespace mc_mujoco
{

/** Configuration for the connection to URLab and the simulation */
struct MjConfiguration
{
  /** ZMQ RPC endpoint of the URLab bridge server (REQ/REP), e.g. "tcp://localhost:5559" */
  std::string urlab_endpoint = "tcp://localhost:5559";
  /** Timeout (ms) for any single URLab RPC round-trip */
  int urlab_timeout_ms = 6000;
  /** Timeout (s) given to begin_pie() when PIE is not already running */
  double urlab_pie_timeout_s = 30.0;
  /** If true, enable the mc_rtc GUI window (2D ImGui panel only; URLab renders the 3D scene) */
  bool with_mc_rtc_gui = true;
  /** If true, enable the mc_rtc controller */
  bool with_controller = true;
  /** If true, sync simulation time and real time */
  bool sync_real_time = false;
  /** If true, start in step-by-step mode */
  bool step_by_step = false;
  /** mc_rtc configuration file */
  std::string mc_config = "";
  /** Use torque-control rather than position control */
  bool torque_control = false;
};

} // namespace mc_mujoco
