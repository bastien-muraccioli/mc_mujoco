#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include "mujoco.h"

#include "mj_configuration.h"

namespace mc_mujoco
{

struct MjSimImpl;

struct MjSim
{
public:
  /*! \brief Constructor
   *
   * Connects to the URLab bridge server, performs the handshake, loads the resulting model locally, and
   * prepares to start a simulation.
   *
   * \param config Configuration for mc_mujoco
   *
   */
  MjSim(const MjConfiguration & config);

  /*! \brief Destructor */
  ~MjSim();

  /** Reset the simulation and controller to the given state */
  void resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs = {},
                       const std::map<std::string, sva::PTransformd> & reset_pos = {});

  /** Sends one URLab step() RPC (advancing frameskip_ physics steps) and runs one mc_rtc control tick.
   * Should be called as often as possible; pacing against URLab's own physics rate is not necessary
   * since "direct" step mode blocks until the requested steps have completed server-side.
   *
   * \returns True if the controller fails and the simulation should stop
   */
  bool stepSimulation();

  /*! Stop the simulation */
  void stopSimulation();

  /*! Update and draw the mc_rtc GUI window (2D ImGui panel only -- no 3D scene, URLab renders the robot
   * and environment in Unreal Engine). No-op if the GUI is disabled.
   *
   * \returns False if the application should quit
   */
  bool render();

  /** The underlying global controller instance in the simulation
   *
   * nullptr if with_controller was false in MjConfiguration
   */
  mc_control::MCGlobalController * controller() noexcept;

  /** Return the local MuJoCo model mirror (loaded from URLab's handshake mjb) */
  mjModel & model() noexcept;

  /** Return the local MuJoCo data mirror (kinematics only, written from URLab's step replies; never
   * locally integrated) */
  mjData & data() noexcept;

private:
  std::unique_ptr<MjSimImpl> impl;
};

} // namespace mc_mujoco
