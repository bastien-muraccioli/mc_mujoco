// Standalone downstream-usage smoke test for the mc_mujoco::mc_mujoco CMake target.
//
// This intentionally does NOT construct mc_mujoco::MjSim: doing so now performs a live ZMQ handshake
// against a URLab bridge server (MjSim's constructor calls URLabClient::hello()/beginPIE()), which is
// not available in a CI environment that only has MuJoCo and mc_rtc installed. Exercising that path
// requires an actual URLab/Unreal session and belongs in an integration test against a real (or mocked)
// bridge server, not this "can a downstream project find_package(mc_mujoco) and link against it" check.
//
// What this still verifies: the installed headers are self-contained and the installed library links,
// which is what downstream consumers (e.g. a project depending on mc_mujoco::mc_mujoco for its
// MjConfiguration/MjSim types) actually need from CI.
#include <mc_mujoco/mj_configuration.h>
#include <mc_mujoco/mj_sim.h>

#include <mc_rtc/logging.h>

int main()
{
  mc_mujoco::MjConfiguration config;
  config.urlab_endpoint = "tcp://localhost:5559";
  config.with_mc_rtc_gui = false;
  config.with_controller = false;
  mc_rtc::log::info("mc_mujoco::MjConfiguration constructed OK, urlab_endpoint={}", config.urlab_endpoint);
  return 0;
}
