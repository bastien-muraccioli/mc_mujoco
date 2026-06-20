#pragma once

#include "mj_sim.h"

#include "MujocoClient.h"
#include "urlab_client.h"

#include "mujoco.h"

#include "glfw3.h"

#include <condition_variable>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mc_mujoco
{

using duration_ms = std::chrono::duration<double, std::milli>;
using duration_us = std::chrono::duration<double, std::micro>;

using clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady,
                                 std::chrono::high_resolution_clock,
                                 std::chrono::steady_clock>;

/** Non-articulated entity tracked in URLab (e.g. a free-floating prop) that does not exist in mc_rtc.
 *
 * URLab reports these under the "entities" block of step/reset replies. We currently only need to read
 * their pose back for datastore consumers; mc_mujoco does not spawn these (URLab's scene authoring owns
 * that), it only forwards what the server reports.
 */
struct MjObject
{
  /** Name in URLab's "entities" map (== UE actor name) */
  std::string name;
  /** Last known pose in world frame, as reported by URLab */
  sva::PTransformd posW = sva::PTransformd::Identity();
};

/** Interface between a URLab articulation and an mc_rtc robot
 *
 * Unlike the original local-MuJoCo implementation, this struct does not index into a locally-stepped
 * mjData by integer id: URLab is the physics authority, and all state crosses the wire by name. The
 * local mjModel/mjData (loaded once from the handshake's mjb) is kept purely as a kinematic/sensor
 * bookkeeping mirror: we write URLab's reported qpos/qvel into it and call mj_forward (never mj_step) so
 * mc_rtc can read mj-derived quantities (sensor addresses, qfrc_actuator, etc.) the same way it always
 * has.
 */
struct MjRobot
{
  /** Name in mc_rtc */
  std::string name;
  /** Prefix used by this articulation in URLab (== MJCF model name); key into URLabStepResult's
   * per_articulation map and into outgoing per_articulation_ctrl requests */
  std::string prefix;
  /** Root body name in MuJoCo */
  std::string root_body;
  /** Root body id (in the local mjModel mirror) */
  int root_body_id = -1;
  /** Free joint in MuJoCo */
  std::string root_joint;
  /** Root joint type */
  mjtJoint root_joint_type = mjJNT_FREE;
  /** Index of robot's root in qpos, -1 if fixed base */
  int root_qpos_idx = -1;
  /** Index of robot's root in qvel, -1 if fixed base */
  int root_qvel_idx = -1;
  /** Position of FloatingBase sensor */
  Eigen::Vector3d root_pos;
  /** Orientation of FloatingBase sensor */
  Eigen::Quaterniond root_ori;
  /** Linear velocity of FloatingBase sensor */
  Eigen::Vector3d root_linvel;
  /** Angular velocity of FloatingBase sensor */
  Eigen::Vector3d root_angvel;
  /** Linear acceleration of FloatingBase sensor */
  Eigen::Vector3d root_linacc;
  /** Angular acceleration of FloatingBase sensor */
  Eigen::Vector3d root_angacc;
  /** Encoders in robot.ref_joint_order */
  std::vector<double> encoders;
  /** Joints' velocity in robot.ref_joint_order */
  std::vector<double> alphas;
  /** Joints' torque in robot.ref_joint_order */
  std::vector<double> torques;
  /** Force sensors reading */
  std::map<std::string, sva::ForceVecd> wrenches;
  /** Gyro readings */
  std::map<std::string, Eigen::Vector3d> gyros;
  /** Accelerometer readings */
  std::map<std::string, Eigen::Vector3d> accelerometers;

  /** Proportional gains for low-level PD control (read from file) */
  std::vector<double> default_kp = {};
  /** Derivative gains for low-level PD control (read from file) */
  std::vector<double> default_kd = {};
  /** Proportional gains for low-level PD control (used in PD loop) */
  std::vector<double> kp = {};
  /** Derivative gains for low-level PD control (used in PD loop) */
  std::vector<double> kd = {};

  /** Live (possibly UE-renamed) MuJoCo joint name for each entry of the reference joint order, empty if
   * absent in the model (mirrors the old mj_jnt_names, but only ever used to resolve ids in the local
   * mjModel mirror for sensor/qpos bookkeeping -- never to address a locally-stepped mjData) */
  std::vector<std::string> mj_jnt_names;
  /** Correspondance from joint name to id inside the local mjModel mirror */
  std::vector<int> mj_jnt_ids;
  /** MuJoCo joint to rjo index */
  std::vector<int> mj_jnt_to_rjo;
  /** Live actuator name (as URLab will accept in a ctrl_map) for each reference-joint-order entry, empty
   * if that joint has no actuator in the model */
  std::vector<std::string> mj_act_names;
  /** Correspondance from mc_rtc force sensor's name to MuJoCo force sensor id (local mirror), -1 if absent */
  std::unordered_map<std::string, int> mc_fs_to_mj_fsensor_id;
  /** Correspondance from mc_rtc force sensor's name to MuJoCo torque sensor id (local mirror), -1 if absent */
  std::unordered_map<std::string, int> mc_fs_to_mj_tsensor_id;
  /** Correspondance from mc-rtc body sensor's name to MuJoCo gyro sensor id (local mirror), -1 if absent */
  std::unordered_map<std::string, int> mc_bs_to_mj_gyro_id;
  /** Correspondance from mc-rtc body sensor's name to MuJoCo accelerometer sensor id (local mirror), -1
   * if absent */
  std::unordered_map<std::string, int> mc_bs_to_mj_accelerometer_id;

  /** Transform from index in mj_jnt_names/mj_act_names to index in mbc, -1 if not in mbc */
  std::vector<int> mj_to_mbc;
  /** Previous position desired by mc_rtc */
  std::vector<double> mj_prev_ctrl_q;
  /** Previous velocity desired by mc_rtc */
  std::vector<double> mj_prev_ctrl_alpha;
  /** Previous torque desired by mc_rtc */
  std::vector<double> mj_prev_ctrl_jointTorque;
  /** Next position desired by mc_rtc */
  std::vector<double> mj_next_ctrl_q;
  /** Next velocity desired by mc_rtc */
  std::vector<double> mj_next_ctrl_alpha;
  /** Next torque desired by mc_rtc */
  std::vector<double> mj_next_ctrl_jointTorque;

  /** Initialize some data after the local mjModel mirror has been loaded */
  void initialize(mjModel * model, const mc_rbdyn::Robot & robot);

  /** Reset the state based on the mc_rtc robot state */
  void reset(const mc_rbdyn::Robot & robot);

  /** Update encoders/alphas/torques and mc_rtc sensors from a URLab step/reset reply, then push the
   * resulting qpos/qvel into the local mjData mirror (caller is responsible for calling mj_forward
   * afterwards once all robots have been updated). */
  void updateSensors(mc_control::MCGlobalController * gc,
                     mjModel * model,
                     mjData * data,
                     const URLabArticulationObs & obs);

  /** Update the control setpoint from the mc_rtc robot state (called once per control tick, i.e. once
   * every frameskip_ physics steps) */
  void updateControl(const mc_rbdyn::Robot & robot);

  /** Compute this sub-step's command (PD or direct torque) and write it into out_ctrl_map, keyed by live
   * actuator name, ready to ship in a URLab step() request. */
  void sendControl(std::map<std::string, double> & out_ctrl_map,
                   size_t interp_idx,
                   size_t frameskip_,
                   bool torque_control);

  /** Run PD control for a given joint */
  double PD(size_t jnt_id, double q_ref, double q, double qdot_ref, double qdot);

  /** Load PD gains from a file */
  bool loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints);

  /** From a name returns the prefixed name in MuJoCo */
  inline std::string prefixed(const std::string & name) const noexcept
  {
    if(!prefix.empty())
    {
      return fmt::format("{}_{}", prefix, name);
    }
    return name;
  }
};

struct MjSimImpl
{
private:
  /** Controller instance in this simulation, might be null if the controller is disabled */
  std::unique_ptr<mc_control::MCGlobalController> controller;

public:
  /** Configuration and data for the step-by-step mode */
  MjConfiguration config;

  /** mc_rtc GUI panel client (2D ImGui only), null if the GUI is disabled */
  std::unique_ptr<MujocoClient> client;

  /** Connection to the URLab bridge server. This is the physics authority: mc_mujoco never steps
   * physics locally, it only sends control and reads back state through this client. */
  std::unique_ptr<URLabClient> urlab;

  /** Local MuJoCo model, loaded once from URLab's handshake mjb. Used purely for kinematic/sensor
   * bookkeeping (name/id resolution, mj_forward to populate derived quantities mc_rtc reads); never
   * passed to mj_step. */
  mjModel * model = nullptr;

  /** Local MuJoCo data mirror. Written from URLab's step replies (qpos/qvel), then mj_forward'd; never
   * integrated locally. */
  mjData * data = nullptr;

  /** GLFW window for the mc_rtc GUI panel (2D ImGui only -- no 3D MuJoCo scene is rendered here, URLab
   * renders the robot and scene in Unreal Engine). May be null if the GUI is disabled. */
  GLFWwindow * window = nullptr;

  /** Start of the previous iteration */
  clock::time_point mj_sim_start_t;
  /** Accumulated delay to catch up to real-time performace */
  duration_us mj_sync_delay = duration_us(0);
  /** Time taken for the last 1024 iterations */
  std::array<double, 1024> mj_sim_dt;
  /** Average of the last 1024 iterations */
  double mj_sim_dt_average;

  /** Number of steps left to play in step by step mode */
  size_t rem_steps = 0;

  /** Robots in simulation and mc_rtc */
  std::vector<MjRobot> robots;

  /** Non-articulated entities reported by URLab (read-only mirror, see MjObject) */
  std::vector<MjObject> objects;

  /*! Simulation wall clock time (seconds), mirrors URLab's reported step_ok "time" field */
  double wallclock = 0.0;

private:
  /** Number of control-relevant simulation iterations since the start (i.e. number of step() RPCs sent
   * to URLab, NOT raw MuJoCo physics steps -- the server already does frameskip_ steps per call). */
  size_t iterCount_ = 0;
  /** How often we run mc_rtc relative to MuJoCo physics. Each URLab step() RPC advances frameskip_
   * physics steps and we run exactly one mc_rtc control tick per RPC. */
  size_t frameskip_ = 1;

  /** True if the simulation should be reset on the next step */
  bool reset_simulation_ = false;

public:
  MjSimImpl(const MjConfiguration & config);

  void cleanup();

  void makeDatastoreCalls();

  void startSimulation();

  /** Apply a URLab step/reset reply to the local state: per-robot updateSensors, then mj_forward to
   * populate derived quantities, then push values into mc_rtc. */
  void updateData(const URLabStepResult & result);

  bool controlStep(std::map<std::string, std::map<std::string, double>> & out_per_articulation_ctrl);

  /** Send one URLab step() RPC (frameskip_ physics steps) and apply the reply */
  void simStep();

  bool stepSimulation();

  bool render();

  void stopSimulation();

  void resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs,
                       const std::map<std::string, sva::PTransformd> & reset_pos);

  // Set the position of an object (no-op: object pose is authoritative in URLab; the SetPosW datastore
  // calls are not meaningful when mc_mujoco does not own physics. Logged once if invoked.)
  void setObjectPosW(const std::string & object, const sva::PTransformd & pt);

  // Retrieve the last known position of an object as reported by URLab
  // Throws if the object does not exist in the simulation
  sva::PTransformd getObjectPosW(const std::string & object) const;

  // Set the position of a robot (no-op for the same reason as setObjectPosW; logged once if invoked)
  void setRobotPosW(const std::string & robot, const sva::PTransformd & pt);

  void saveGUISettings();

  inline mc_control::MCGlobalController * get_controller() noexcept
  {
    return controller.get();
  }

  std::map<std::string, std::vector<double>> init_qs_;
  std::map<std::string, sva::PTransformd> init_pos_;
};

} // namespace mc_mujoco
