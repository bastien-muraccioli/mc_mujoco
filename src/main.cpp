#include "mj_sim.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <cmath>
#include <iostream>
#include <thread>

#include <CLI/CLI.hpp>

bool render_state = true;

void simulate(mc_mujoco::MjSim & mj_sim)
{
  bool done = false;
  while(!done && render_state)
  {
    done = mj_sim.stepSimulation();
  }
}

int main(int argc, char * argv[])
{
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("mc_mujoco was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile mc_mujoco",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  mc_mujoco::MjConfiguration config;

  CLI::App app{"mc_mujoco options (URLab bridge mode: physics runs in Unreal Engine/URLab, mc_mujoco only "
               "computes control and the mc_rtc GUI)"};

  app.add_option("mc-config,-f,--mc-config", config.mc_config, "Configuration given to mc_rtc");
  app.add_option("--urlab-endpoint", config.urlab_endpoint,
                 "URLab bridge server ZMQ RPC endpoint (default: tcp://localhost:5559)");
  app.add_option("--urlab-timeout-ms", config.urlab_timeout_ms, "Timeout (ms) for a single URLab RPC round-trip");
  app.add_option("--urlab-pie-timeout-s", config.urlab_pie_timeout_s,
                 "Timeout (s) to wait for URLab to enter Play-In-Editor if it is not already running");

  app.add_flag("--step-by-step", config.step_by_step, "Start the simulation in step-by-step mode");
  app.add_flag("--torque-control", config.torque_control, "Enable torque control");
  app.add_flag("-s,--sync", config.sync_real_time, "Synchronize mc_mujoco simulation time with real time");

  app.add_flag("--without-controller{false}", config.with_controller, "Disable mc_rtc controller");
  app.add_flag("--without-mc-rtc-gui{false}", config.with_mc_rtc_gui, "Disable the mc_rtc GUI window");

  CLI11_PARSE(app, argc, argv);

  mc_mujoco::MjSim mj_sim(config);

  std::thread simThread(simulate, std::ref(mj_sim));

  while(render_state)
  {
    render_state = mj_sim.render();
  }

  simThread.join();
  mj_sim.stopSimulation();
  return 0;
}
