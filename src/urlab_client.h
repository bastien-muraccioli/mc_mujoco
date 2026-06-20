#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <msgpack.hpp>
#include <zmq.hpp>

namespace mc_mujoco
{

/** Per-articulation observation returned by a step/reset/forward reply
 *
 * Only the fields relevant to driving an mc_rtc controller are kept: full joint state and named sensor
 * readings. Camera, body xpos/xquat and other "full" level fields are not used by this bridge.
 */
struct URLabArticulationObs
{
  /** Generalized positions (nq), compiled MuJoCo order */
  std::vector<double> qpos;
  /** Generalized velocities (nv), compiled MuJoCo order */
  std::vector<double> qvel;
  /** Last applied control (nu), compiled MuJoCo order */
  std::vector<double> ctrl;
  /** Named sensor readings, flattened (gyro -> 3 doubles, force -> 3 doubles, etc.) */
  std::map<std::string, std::vector<double>> sensors;
};

/** One entry of the handshake's "articulations" block
 *
 * Only the fields this bridge needs are kept; see the URLab protocol reference for the full block.
 */
struct URLabArticulationInfo
{
  /** Prefix used for this articulation's entities in the compiled model (== MJCF model name) */
  std::string prefix;
  /** Stable actor id assigned when the actor was spawned */
  std::string actor_id;
  /** "ue_controller" if a controller component is attached server-side, else "raw" */
  std::string default_control_mode;
  /** original (authored XML) -> live (possibly UE-renamed) name, per category: "actuators", "joints",
   * "sensors", "bodies". We only need "actuators" and "joints" here. */
  std::map<std::string, std::map<std::string, std::string>> original_names;
};

/** Result of a successful hello() handshake */
struct URLabHandshake
{
  std::string session_id;
  std::string urlab_version;
  std::string mujoco_version;
  int mujoco_version_int = 0;
  bool manager_present = false;
  /** Compiled MuJoCo model (mjb format), to be loaded locally with mj_loadModel */
  std::string mjb;
  std::vector<URLabArticulationInfo> articulations;
};

/** Result of a successful step()/reset()/forward() call */
struct URLabStepResult
{
  double time = 0.0;
  long long step = 0;
  /** Per articulation, keyed by prefix (== handshake's "prefix" field, the MJCF model name) */
  std::map<std::string, URLabArticulationObs> per_articulation;
};

/** Thrown when the server replies with an "error" op, or on protocol-level failures
 * (malformed reply, transport error, timeout). */
class URLabError : public std::runtime_error
{
public:
  URLabError(const std::string & code, const std::string & message)
  : std::runtime_error("[URLab] " + code + ": " + message), code_(code)
  {
  }

  const std::string & code() const noexcept
  {
    return code_;
  }

private:
  std::string code_;
};

/** Minimal C++ client for URLab's ZMQ RPC protocol (REQ/REP, msgpack-encoded)
 *
 * Implements exactly the lifecycle mc_mujoco needs to run as an external controller driving URLab's
 * physics: hello (handshake + model retrieval), begin_pie (enter Play-In-Editor if not already running),
 * step (direct mode, one or more physics steps against an explicit per-articulation ctrl), and reset.
 *
 * This intentionally does not implement the full protocol (cameras, recording, replay, editor scene
 * authoring, puppet/live modes, ...) since mc_mujoco only ever drives physics in "direct" step mode with
 * control_mode "raw" (mc_rtc computes the actuator command itself; URLab must not also run its own
 * UE-side PD controller on top).
 */
class URLabClient
{
public:
  /** Connect to the URLab bridge server
   *
   * \param endpoint REQ/REP endpoint, e.g. "tcp://localhost:5559"
   *
   * \param timeout_ms Receive timeout for any single RPC round-trip. URLab's own step_timeout is 5s
   * server-side; we default a little above that.
   */
  explicit URLabClient(const std::string & endpoint, int timeout_ms = 6000);

  ~URLabClient();

  URLabClient(const URLabClient &) = delete;
  URLabClient & operator=(const URLabClient &) = delete;

  /** Perform the hello handshake
   *
   * Requests "standard" observation level (qpos/qvel/ctrl/act/sensors), msgpack encoding, and does not
   * request asset bytes (include_assets=false): mc_mujoco never needs meshes/textures, only the compiled
   * model for kinematic/sensor bookkeeping.
   *
   * Throws URLabError on failure.
   */
  URLabHandshake hello();

  /** Ensure the editor is in Play-In-Editor (PIE) mode, i.e. that a manager is active and physics can be
   * stepped. No-op (returns immediately) if a manager is already present from hello().
   *
   * On success, re-absorbs the refreshed handshake embedded in the begin_pie reply (articulations are
   * only populated once PIE is running).
   *
   * Throws URLabError on failure (including compile_failed/timeout).
   */
  URLabHandshake beginPIE(double timeout_s = 30.0);

  /** Advance the simulation by n_steps in direct mode
   *
   * \param per_articulation_ctrl Map from articulation prefix to the named actuator control map
   * (live/UE actuator name -> value) to apply this step window. control_mode is forced to "raw" for
   * every articulation present: mc_rtc is the only controller, URLab must not also apply its own
   * UE-side PD on these actuators.
   *
   * \param n_steps Number of MuJoCo physics steps to advance.
   *
   * Throws URLabError on failure.
   */
  URLabStepResult step(const std::map<std::string, std::map<std::string, double>> & per_articulation_ctrl,
                       size_t n_steps = 1);

  /** Reset the simulation
   *
   * \param keyframe_name Optional MJCF keyframe to reset to (mj_resetDataKeyframe). Empty for a plain
   * mj_resetData.
   *
   * \param per_articulation_qpos Optional named joint -> value overrides applied after the reset.
   *
   * Throws URLabError on failure.
   */
  URLabStepResult reset(const std::string & keyframe_name = "",
                        const std::map<std::string, std::map<std::string, double>> & per_articulation_qpos = {});

  /** Patch a controller's gains/params server-side. Used for the SetPDGains datastore calls.
   *
   * Best-effort: logs and returns false rather than throwing, since this is called from the simulation
   * hot path via datastore callbacks where a single missed gain update should not stop the simulation.
   */
  bool configureController(const std::string & articulation,
                           const std::map<std::string, double> & kp,
                           const std::map<std::string, double> & kd);

  /** session_id from the last successful hello()/beginPIE(), empty if hello() was never called */
  const std::string & sessionId() const noexcept
  {
    return session_id_;
  }

private:
  /** Send a msgpack-encoded request object and decode the reply into a generic handle.
   *
   * Adds "session_id" automatically (once one is known) unless the request already set one. Throws
   * URLabError on a transport failure, a non-"<op>_ok" reply, or an explicit "error" reply.
   */
  msgpack::object_handle call(const std::string & op,
                              const msgpack::sbuffer & extra_fields_packed,
                              const std::string & expect_reply_op);

  /** Parse the "articulations" array of a hello_ok/begin_pie reply into our struct */
  static std::vector<URLabArticulationInfo> parseArticulations(const msgpack::object & articulations_arr);

  /** Parse a step_ok/reset_ok/forward_ok reply's body into a URLabStepResult */
  static URLabStepResult parseStepResult(const msgpack::object & reply);

  std::string endpoint_;
  int timeout_ms_;
  std::unique_ptr<zmq::context_t> ctx_;
  std::unique_ptr<zmq::socket_t> sock_;
  std::string session_id_;
};

} // namespace mc_mujoco
