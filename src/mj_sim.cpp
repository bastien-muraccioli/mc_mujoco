#include "mj_sim_impl.h"
#include "mj_utils.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <fstream>
#include <set>
#include <sstream>
#include <type_traits>

#include "MujocoClient.h"
#include "config.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot.h"

#include <filesystem>
namespace fs = std::filesystem;

#include <mc_rtc/version.h>

namespace mc_mujoco
{

double MjRobot::PD(size_t jnt_id, double q_ref, double q, double qdot_ref, double qdot)
{
  double p_error = q_ref - q;
  double v_error = qdot_ref - qdot;
  double ret = (kp[jnt_id] * p_error + kd[jnt_id] * v_error);
  return ret;
}

/* Load PD gains from file (taken from RobotHardware/robot.cpp) */
bool MjRobot::loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints)
{
  std::ifstream strm(path_to_pd.c_str());
  if(!strm.is_open())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Cannot open PD gains file for {} at {}", name,
                                                     path_to_pd);
  }

  size_t num_joints = joints.size();
  if(!num_joints)
  {
    return false;
  }
  std::vector<double> default_pgain(num_joints, 0);
  std::vector<double> default_dgain(num_joints, 0);
  for(int i = 0; i < num_joints; i++)
  {
    std::string str;
    bool getlinep;
    while((getlinep = !!(std::getline(strm, str))))
    {
      if(str.empty())
      {
        continue;
      }
      if(str[0] == '#')
      {
        continue;
      }
      double tmp;
      std::istringstream sstrm(str);
      sstrm >> tmp;
      default_pgain[i] = tmp;
      if(sstrm.eof()) break;

      sstrm >> tmp;
      default_dgain[i] = tmp;
      if(sstrm.eof()) break;
      break;
    }
    if(!getlinep)
    {
      if(i < num_joints)
      {
        mc_rtc::log::error(
            "[mc_mujoco] loadGain error: size of gains reading from file ({}) does not match size of joints",
            path_to_pd);
      }
      break;
    }
  }

  strm.close();
  mc_rtc::log::info("[mc_mujoco] Gains for {}", name);
  for(unsigned int i = 0; i < num_joints; i++)
  {
    mc_rtc::log::info("[mc_mujoco] {}, pgain = {}, dgain = {}", joints[i], default_pgain[i], default_dgain[i]);
    default_kp.push_back(default_pgain[i]);
    default_kd.push_back(default_dgain[i]);
    kp.push_back(default_pgain[i]);
    kd.push_back(default_dgain[i]);
  }
  return true;
}

MjSimImpl::MjSimImpl(const MjConfiguration & config)
: controller(std::make_unique<mc_control::MCGlobalController>(config.mc_config)), config(config)
{
  auto get_robot_cfg_path_local = [&](const std::string & robot_name)
  { return fs::path(mc_mujoco::USER_FOLDER) / (robot_name + ".yaml"); };
  auto get_robot_cfg_path_global = [&](const std::string & robot_name)
  { return fs::path(mc_mujoco::SHARE_FOLDER) / (robot_name + ".yaml"); };
  auto get_robot_cfg_path = [&](const std::string & robot_name) -> std::string
  {
    if(fs::exists(get_robot_cfg_path_local(robot_name)))
    {
      return get_robot_cfg_path_local(robot_name).string();
    }
    if(fs::exists(get_robot_cfg_path_global(robot_name)))
    {
      return get_robot_cfg_path_global(robot_name).string();
    }
    return "";
  };

  // We still read each mc_rtc robot's <robot>.yaml for pdGainsPath: PD gains remain mc_mujoco/mc_rtc's
  // responsibility even though the model geometry itself now comes from URLab. xmlModelPath (if present)
  // is ignored: URLab is the sole source of the compiled model.
  std::map<std::string, std::string> pdGainsFiles;
  for(const auto & r : controller->robots())
  {
    const auto & robot_cfg_path = get_robot_cfg_path(r.module().name);
    if(robot_cfg_path.empty())
    {
      continue;
    }
    auto robot_cfg = mc_rtc::Configuration(robot_cfg_path);

    auto main_robot_params = [&]() -> std::vector<std::string>
    {
      auto main_robot_cfg = controller->configuration().config.find("MainRobot");
      if(!main_robot_cfg)
      {
        return {"JVRC1"};
      }
      if(main_robot_cfg->isArray())
      {
        return main_robot_cfg->operator std::vector<std::string>();
      }
      if(main_robot_cfg->isObject())
      {
        auto module_cfg = (*main_robot_cfg)("module");
        if(module_cfg.isArray())
        {
          return module_cfg.operator std::vector<std::string>();
        }
        return {module_cfg.operator std::string()};
      }
      return {main_robot_cfg->operator std::string()};
    }();
    const auto & main_robot_name = main_robot_params[0];

    std::string pdGainsPath = "";
    if(!main_robot_name.empty() && robot_cfg.find(main_robot_name) && robot_cfg(main_robot_name).find("pdGainsPath"))
    {
      pdGainsPath = robot_cfg(main_robot_name)("pdGainsPath", std::string(""));
    }
    else if(robot_cfg.find(r.name().c_str()) && robot_cfg(r.name().c_str()).find("pdGainsPath")
            && !robot_cfg(r.name().c_str())("pdGainsPath", std::string("")).empty())
    {
      pdGainsPath = robot_cfg(r.name().c_str())("pdGainsPath", std::string(""));
    }
    else if(robot_cfg.find("pdGainsPath"))
    {
      pdGainsPath = robot_cfg("pdGainsPath", std::string(""));
    }
    pdGainsFiles[r.name()] = pdGainsPath;
  }

  // Connect to URLab and load the model it reports.
  urlab = std::make_unique<URLabClient>(config.urlab_endpoint, config.urlab_timeout_ms);
  bool initialized = mujoco_init(this);
  if(!initialized)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Initialized failed.");
  }

  // read PD gains from file
  for(size_t i = 0; i < robots.size(); ++i)
  {
    auto & r = robots[i];
    bool has_motor =
        std::any_of(r.mj_act_names.begin(), r.mj_act_names.end(), [](const std::string & m) { return !m.empty(); });
    const auto & robot = controller->robot(r.name);
    if(robot.mb().nrDof() == 0 || (robot.mb().nrDof() == 6 && robot.mb().joint(0).dof() == 6) || !has_motor)
    {
      continue;
    }
    if(pdGainsFiles.count(r.name) == 0 || !fs::exists(pdGainsFiles[r.name]))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] PD gains file for {} cannot be found at {}", r.name,
                                                       pdGainsFiles[r.name]);
    }
    r.loadGain(pdGainsFiles[r.name], controller->robots().robot(r.name).module().ref_joint_order());
  }

  if(config.with_mc_rtc_gui)
  {
    mujoco_create_window(this);
    client = std::make_unique<MujocoClient>();
  }
  mc_rtc::log::info("[mc_mujoco] Initialized successful.");
}

void MjSimImpl::cleanup()
{
  mujoco_cleanup(this);
}

void MjRobot::initialize(mjModel * model, const mc_rbdyn::Robot & robot)
{
  mj_jnt_ids.resize(0);
  for(const auto & j : mj_jnt_names)
  {
    mj_jnt_ids.push_back(j.empty() ? -1 : mj_name2id(model, mjOBJ_JOINT, j.c_str()));
  }
  if(!root_body.empty())
  {
    root_body_id = mj_name2id(model, mjOBJ_BODY, root_body.c_str());
  }
  if(!root_joint.empty())
  {
    auto root_joint_id = mj_name2id(model, mjOBJ_JOINT, root_joint.c_str());
    root_qpos_idx = model->jnt_qposadr[root_joint_id];
    root_qvel_idx = model->jnt_dofadr[root_joint_id];
  }
  auto init_sensor_id = [&](const char * mj_name, const char * mc_name, const std::string & sensor_name,
                            const char * suffix, mjtSensor type, std::unordered_map<std::string, int> & mapping)
  {
    auto mj_sensor = prefixed(fmt::format("{}_{}", sensor_name, suffix));
    auto sensor_id = mujoco_get_sensor_id(*model, mj_sensor, type);
    if(sensor_id == -1)
    {
      mc_rtc::log::error("[mc_mujoco] No MuJoCo {} for {} {} in {}, expected to find a {} named {}", mj_name,
                         sensor_name, mc_name, name, mj_name, mj_sensor);
    }
    mapping[sensor_name] = sensor_id;
  };
  for(const auto & fs : robot.module().forceSensors())
  {
    wrenches[fs.name()] = sva::ForceVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    init_sensor_id("force sensor", "force sensor", fs.name(), "fsensor", mjSENS_FORCE, mc_fs_to_mj_fsensor_id);
    init_sensor_id("torque sensor", "force sensor", fs.name(), "tsensor", mjSENS_TORQUE, mc_fs_to_mj_tsensor_id);
  }
  for(const auto & bs : robot.bodySensors())
  {
    if(bs.name() == "FloatingBase" || bs.name().empty())
    {
      continue;
    }
    gyros[bs.name()] = Eigen::Vector3d::Zero();
    accelerometers[bs.name()] = Eigen::Vector3d::Zero();
    init_sensor_id("gyro sensor", "body sensor", bs.name(), "gyro", mjSENS_GYRO, mc_bs_to_mj_gyro_id);
    init_sensor_id("accelerometer sensor", "body sensor", bs.name(), "accelerometer", mjSENS_ACCELEROMETER,
                   mc_bs_to_mj_accelerometer_id);
  }
  reset(robot);
}

void MjRobot::reset(const mc_rbdyn::Robot & robot)
{
  const auto & rjo = robot.module().ref_joint_order();
  if(rjo.size() != mj_jnt_names.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_mujoco] Missmatch in model for {}, reference joint order has {} joints but MuJoCo model has {} joints",
        name, rjo.size(), mj_jnt_names.size());
  }
  mj_to_mbc.resize(0);
  mj_prev_ctrl_q.resize(0);
  mj_prev_ctrl_alpha.resize(0);
  mj_prev_ctrl_jointTorque.resize(0);
  mj_jnt_to_rjo.resize(0);
  encoders = std::vector<double>(rjo.size(), 0.0);
  alphas = std::vector<double>(rjo.size(), 0.0);
  torques = std::vector<double>(rjo.size(), 0.0);
  for(const auto & mj_jn : mj_jnt_names)
  {
    const auto & jn = [&]()
    {
      if(!prefix.empty() && mj_jn.size() > prefix.size() + 1)
      {
        return mj_jn.substr(prefix.size() + 1);
      }
      return mj_jn;
    }();
    auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
    int rjo_idx = -1;
    if(rjo_it != rjo.end())
    {
      rjo_idx = std::distance(rjo.begin(), rjo_it);
    }
    mj_jnt_to_rjo.push_back(rjo_idx);
    if(robot.hasJoint(jn))
    {
      auto jIndex = robot.jointIndexByName(jn);
      mj_to_mbc.push_back(jIndex);
      if(robot.mb().joint(jIndex).dof() != 1)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[mc_mujoco] Only support revolute and prismatic joint for control");
      }
      mj_prev_ctrl_q.push_back(robot.mbc().q[jIndex][0]);
      mj_prev_ctrl_alpha.push_back(robot.mbc().alpha[jIndex][0]);
      mj_prev_ctrl_jointTorque.push_back(robot.mbc().jointTorque[jIndex][0]);
      if(rjo_idx != -1)
      {
        encoders[rjo_idx] = mj_prev_ctrl_q.back();
        alphas[rjo_idx] = mj_prev_ctrl_alpha.back();
        torques[rjo_idx] = mj_prev_ctrl_jointTorque.back();
      }
    }
    else
    {
      mj_to_mbc.push_back(-1);
    }
  }
  mj_next_ctrl_q = mj_prev_ctrl_q;
  mj_next_ctrl_alpha = mj_prev_ctrl_alpha;
  mj_next_ctrl_jointTorque = mj_prev_ctrl_jointTorque;

  // reset the PD gains to default values
  kp = default_kp;
  kd = default_kd;
}

sva::PTransformd MjSimImpl::getObjectPosW(const std::string & object) const
{
  auto it = std::find_if(objects.begin(), objects.end(), [&](const auto & o) { return o.name == object; });
  if(it == objects.end())
  {
    mc_rtc::log::error_and_throw("Requested position of object {} which is not in this simulation", object);
  }
  return it->posW;
}

void MjSimImpl::setObjectPosW(const std::string & object, const sva::PTransformd &)
{
  // URLab owns physics state; mc_mujoco cannot push an authoritative pose into it (no equivalent
  // "teleport entity" RPC is used by this bridge). Kept as a no-op (rather than removing the datastore
  // call) so existing controllers that call {object}::SetPosW do not crash, with a one-time warning.
  static std::set<std::string> warned;
  if(warned.insert(object).second)
  {
    mc_rtc::log::warning(
        "[mc_mujoco] {}::SetPosW was called, but object poses are authoritative in URLab and cannot be "
        "overridden from mc_mujoco in this bridge configuration. Ignoring.",
        object);
  }
}

void MjSimImpl::setRobotPosW(const std::string & robot, const sva::PTransformd &)
{
  static std::set<std::string> warned;
  if(warned.insert(robot).second)
  {
    mc_rtc::log::warning("[mc_mujoco] {}::SetPosW was called, but robot poses are authoritative in URLab and cannot be "
                         "overridden from mc_mujoco in this bridge configuration. Ignoring.",
                         robot);
  }
}

void MjSimImpl::makeDatastoreCalls()
{
  auto & ds = controller->controller().datastore();
  for(auto & o : objects)
  {
    ds.make_call(o.name + "::SetPosW", [this, name = o.name](const sva::PTransformd & pt) { setObjectPosW(name, pt); });
  }
  for(auto & r : robots)
  {
    ds.make_call(r.name + "::SetPosW", [this, name = r.name](const sva::PTransformd & pt) { setRobotPosW(name, pt); });
    ds.make_call(r.name + "::SetPDGains",
                 [this, &r](const std::vector<double> & p_vec, const std::vector<double> & d_vec)
                 {
                   const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
                   if(p_vec.size() != rjo.size())
                   {
                     mc_rtc::log::warning("[mc_mujoco] {}::SetPDGains failed. p_vec size({})!=ref_joint_order size({})",
                                          r.name, p_vec.size(), rjo.size());
                     return false;
                   }
                   if(d_vec.size() != rjo.size())
                   {
                     mc_rtc::log::warning("[mc_mujoco] {}::SetPDGains failed. d_vec size({})!=ref_joint_order size({})",
                                          r.name, d_vec.size(), rjo.size());
                     return false;
                   }
                   r.kp = p_vec;
                   r.kd = d_vec;
                   return true;
                 });

    ds.make_call(r.name + "::SetPDGainsByName",
                 [this, &r](const std::string & jn, double p, double d)
                 {
                   const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
                   auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
                   if(rjo_it == rjo.end())
                   {
                     mc_rtc::log::warning(
                         "[mc_mujoco] {}::SetPDGainsByName failed. Joint {} not found in ref_joint_order.", r.name, jn);
                     return false;
                   }
                   int rjo_idx = std::distance(rjo.begin(), rjo_it);
                   r.kp[rjo_idx] = p;
                   r.kd[rjo_idx] = d;
                   return true;
                 });

    ds.make_call(r.name + "::GetPDGains",
                 [this, &r](std::vector<double> & p_vec, std::vector<double> & d_vec)
                 {
                   p_vec = r.kp;
                   d_vec = r.kd;
                   const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
                   if(p_vec.size() != rjo.size())
                   {
                     mc_rtc::log::warning("[mc_mujoco] {}::GetPDGains failed. p_vec size({})!=ref_joint_order size({})",
                                          r.name, p_vec.size(), rjo.size());
                     return false;
                   }
                   if(d_vec.size() != rjo.size())
                   {
                     mc_rtc::log::warning("[mc_mujoco] {}::GetPDGains failed. d_vec size({})!=ref_joint_order size({})",
                                          r.name, d_vec.size(), rjo.size());
                     return false;
                   }
                   return true;
                 });

    ds.make_call(r.name + "::GetPDGainsByName",
                 [this, &r](const std::string & jn, double & p, double & d)
                 {
                   const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
                   auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
                   if(rjo_it == rjo.end())
                   {
                     mc_rtc::log::warning(
                         "[mc_mujoco] {}::GetPDGainsByName failed. Joint {} not found in ref_joint_order.", r.name, jn);
                     return false;
                   }
                   int rjo_idx = std::distance(rjo.begin(), rjo_it);
                   p = r.kp[rjo_idx];
                   d = r.kd[rjo_idx];
                   return true;
                 });
  }
}

void MjSimImpl::startSimulation()
{
  if(!config.with_controller)
  {
    controller.reset();
    return;
  }

  makeDatastoreCalls();

  // frameskip_ is computed from the local model's timestep, which mirrors whatever URLab's mjOption.timestep
  // currently is (URLab may override this server-side via set_sim_options; if you change it there, restart
  // mc_mujoco so the local mirror and frameskip stay in sync).
  double simTimestep = model->opt.timestep;
  frameskip_ = std::round(controller->timestep() / simTimestep);
  mc_rtc::log::info("[mc_mujoco] MC-RTC timestep: {}. MJ timestep: {}", controller->timestep(), simTimestep);
  mc_rtc::log::info("[mc_mujoco] Hence, Frameskip: {}", frameskip_);

  for(auto & r : robots)
  {
    r.initialize(model, controller->robot(r.name));
    controller->setEncoderValues(r.name, r.encoders);
  }
  for(const auto & r : robots)
  {
    init_qs_[r.name] = controller->robot(r.name).encoderValues();
    init_pos_[r.name] = controller->controller().robot(r.name).posW();
  }
  controller->init(init_qs_, init_pos_);
  controller->running = true;
}

void MjRobot::updateSensors(mc_control::MCGlobalController * gc,
                            mjModel * model,
                            mjData * data,
                            const URLabArticulationObs & obs)
{
  // Push URLab's reported qpos/qvel into the local mjData mirror so mj_forward can recompute everything
  // derived (xpos, sensordata, qfrc_actuator, ...) the way the original local-stepping code relied on.
  // obs.qpos/obs.qvel are the full compiled-order vectors (same compiled model as our local mirror, see
  // mujoco_init), so jnt_qposadr/jnt_dofadr indices from our local model are valid indices into them.
  for(size_t i = 0; i < mj_jnt_ids.size(); ++i)
  {
    if(mj_jnt_ids[i] == -1)
    {
      continue;
    }
    int adr = model->jnt_qposadr[mj_jnt_ids[i]];
    if(adr < static_cast<int>(obs.qpos.size()))
    {
      data->qpos[adr] = obs.qpos[adr];
    }
  }
  for(size_t i = 0; i < mj_jnt_ids.size(); ++i)
  {
    if(mj_jnt_ids[i] == -1)
    {
      continue;
    }
    int adr = model->jnt_dofadr[mj_jnt_ids[i]];
    if(adr < static_cast<int>(obs.qvel.size()))
    {
      data->qvel[adr] = obs.qvel[adr];
    }
  }

  for(size_t i = 0; i < mj_jnt_ids.size(); ++i)
  {
    if(mj_jnt_to_rjo[i] == -1 || mj_jnt_ids[i] == -1)
    {
      continue;
    }
    encoders[mj_jnt_to_rjo[i]] = data->qpos[model->jnt_qposadr[mj_jnt_ids[i]]];
    alphas[mj_jnt_to_rjo[i]] = data->qvel[model->jnt_dofadr[mj_jnt_ids[i]]];
  }
  // qfrc_actuator is only valid after mj_forward has run on the freshly-written qpos/qvel; the caller
  // (MjSimImpl::updateData) calls mj_forward once for all robots before torques are read out, so we read
  // it here on the *next* updateSensors call (one control-tick of latency, matching the original
  // local-stepping code's qfrc_actuator read which was likewise post-step).
  for(size_t i = 0; i < mj_jnt_ids.size(); ++i)
  {
    if(mj_jnt_to_rjo[i] == -1 || mj_jnt_ids[i] == -1)
    {
      continue;
    }
    torques[mj_jnt_to_rjo[i]] = data->qfrc_actuator[model->jnt_dofadr[mj_jnt_ids[i]]];
  }

  if(!gc)
  {
    return;
  }
  auto & robot = gc->controller().robots().robot(name);

  if(root_qpos_idx != -1)
  {
    root_pos = Eigen::Map<Eigen::Vector3d>(&data->qpos[root_qpos_idx]);
    root_ori.w() = data->qpos[root_qpos_idx + 3];
    root_ori.x() = data->qpos[root_qpos_idx + 4];
    root_ori.y() = data->qpos[root_qpos_idx + 5];
    root_ori.z() = data->qpos[root_qpos_idx + 6];
    root_ori = root_ori.inverse();
    root_linvel = Eigen::Map<Eigen::Vector3d>(&data->qvel[root_qvel_idx]);
    root_angvel = Eigen::Map<Eigen::Vector3d>(&data->qvel[root_qvel_idx + 3]);
    root_linacc = Eigen::Map<Eigen::Vector3d>(&data->qacc[root_qvel_idx]);
    root_angacc = Eigen::Map<Eigen::Vector3d>(&data->qacc[root_qvel_idx + 3]);
    if(robot.hasBodySensor("FloatingBase"))
    {
      gc->setSensorPositions(name, {{"FloatingBase", root_pos}});
      gc->setSensorOrientations(name, {{"FloatingBase", root_ori}});
      gc->setSensorLinearVelocities(name, {{"FloatingBase", root_linvel}});
      gc->setSensorAngularVelocities(name, {{"FloatingBase", root_angvel}});
      gc->setSensorLinearAccelerations(name, {{"FloatingBase", root_linacc}});
    }
  }

  for(auto & gyro : gyros)
  {
    mujoco_get_sensordata(*model, *data, mc_bs_to_mj_gyro_id[gyro.first], gyro.second.data());
  }
  gc->setSensorAngularVelocities(name, gyros);

  for(auto & accelerometer : accelerometers)
  {
    mujoco_get_sensordata(*model, *data, mc_bs_to_mj_accelerometer_id[accelerometer.first],
                          accelerometer.second.data());
  }
  gc->setSensorLinearAccelerations(name, accelerometers);

  for(auto & fs : wrenches)
  {
    mujoco_get_sensordata(*model, *data, mc_fs_to_mj_fsensor_id[fs.first], fs.second.force().data());
    mujoco_get_sensordata(*model, *data, mc_fs_to_mj_tsensor_id[fs.first], fs.second.couple().data());
    fs.second *= -1;
  }
  gc->setWrenches(name, wrenches);

  gc->setEncoderValues(name, encoders);
  gc->setEncoderVelocities(name, alphas);
  gc->setJointTorques(name, torques);
}

void MjSimImpl::updateData(const URLabStepResult & result)
{
  wallclock = result.time;
  // Note: URLabClient::parseStepResult does not currently parse the reply's "entities" block, so
  // non-articulated MjObject poses are not refreshed here and stay at their last-known value. Extend
  // URLabStepResult with an "entities" map (and parse it in updateData) if a controller needs live prop
  // poses via getObjectPosW.
  for(auto & r : robots)
  {
    auto it = result.per_articulation.find(r.prefix);
    if(it == result.per_articulation.end())
    {
      mc_rtc::log::warning("[mc_mujoco] No observation for articulation '{}' (prefix '{}') in step reply", r.name,
                           r.prefix);
      continue;
    }
    r.updateSensors(controller.get(), model, data, it->second);
  }
  // Recompute every quantity derived from qpos/qvel (xpos, sensordata, qfrc_actuator, ...) for the
  // values we just wrote, without integrating: this is the local-mirror equivalent of what mj_step used
  // to do as a side effect, but URLab is the one actually integrating physics.
  mj_forward(model, data);
}

void MjRobot::updateControl(const mc_rbdyn::Robot & robot)
{
  mj_prev_ctrl_q = mj_next_ctrl_q;
  mj_prev_ctrl_alpha = mj_next_ctrl_alpha;
  mj_prev_ctrl_jointTorque = mj_next_ctrl_jointTorque;
  size_t ctrl_idx = 0;
  for(size_t i = 0; i < mj_to_mbc.size(); ++i)
  {
    auto jIndex = mj_to_mbc[i];
    if(jIndex != -1)
    {
      mj_next_ctrl_q[ctrl_idx] = robot.mbc().q[jIndex][0];
      mj_next_ctrl_alpha[ctrl_idx] = robot.mbc().alpha[jIndex][0];
      mj_next_ctrl_jointTorque[ctrl_idx] = robot.mbc().jointTorque[jIndex][0];
      ctrl_idx++;
    }
  }
}

void MjRobot::sendControl(std::map<std::string, double> & out_ctrl_map,
                          size_t interp_idx,
                          size_t frameskip_,
                          bool torque_control)
{
  for(size_t i = 0; i < mj_act_names.size(); ++i)
  {
    if(mj_act_names[i].empty())
    {
      continue;
    }
    auto rjo_id = mj_jnt_to_rjo[i];
    if(rjo_id == -1)
    {
      continue;
    }
    double q_ref = (interp_idx + 1) * (mj_next_ctrl_q[i] - mj_prev_ctrl_q[i]) / frameskip_;
    q_ref += mj_prev_ctrl_q[i];
    double alpha_ref = (interp_idx + 1) * (mj_next_ctrl_alpha[i] - mj_prev_ctrl_alpha[i]) / frameskip_;
    alpha_ref += mj_prev_ctrl_alpha[i];
    double torque_ref = (interp_idx + 1) * (mj_next_ctrl_jointTorque[i] - mj_prev_ctrl_jointTorque[i]) / frameskip_;
    torque_ref += mj_prev_ctrl_jointTorque[i];

    double cmd;
    if(torque_control && torque_ref != 0)
    {
      cmd = torque_ref;
    }
    else
    {
      cmd = PD(rjo_id, q_ref, encoders[rjo_id], alpha_ref, alphas[rjo_id]);
    }
    out_ctrl_map[mj_act_names[i]] = cmd;
  }
}

bool MjSimImpl::controlStep(std::map<std::string, std::map<std::string, double>> & out_per_articulation_ctrl)
{
  auto interp_idx = iterCount_ % frameskip_;
  if(config.with_controller && interp_idx == 0)
  {
    if(!controller->run())
    {
      return true;
    }
    for(auto & r : robots)
    {
      r.updateControl(controller->robots().robot(r.name));
    }
  }
  for(auto & r : robots)
  {
    r.sendControl(out_per_articulation_ctrl[r.prefix], interp_idx, frameskip_, config.torque_control);
  }
  iterCount_++;
  return false;
}

void MjSimImpl::simStep()
{
  std::map<std::string, std::map<std::string, double>> per_articulation_ctrl;
  // controlStep computes one sub-step's command for every robot (interpolated between the previous and
  // next mc_rtc control setpoint). We then send exactly one physics step to URLab per simStep() call so
  // the interpolation stays meaningful; the outer loop (stepSimulation) calls this frameskip_ times per
  // mc_rtc control tick, matching the original local-stepping cadence.
  controlStep(per_articulation_ctrl);
  auto result = urlab->step(per_articulation_ctrl, 1);
  updateData(result);
}

void MjSimImpl::resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs,
                                const std::map<std::string, sva::PTransformd> & reset_pos)
{
  iterCount_ = 0;
  reset_simulation_ = false;
  if(controller)
  {
    controller->reset(reset_qs, reset_pos);
    for(auto & robot : robots)
    {
      robot.reset(controller->robot(robot.name));
    }
    controller->running = true;
  }

  std::map<std::string, std::map<std::string, double>> per_articulation_qpos;
  for(const auto & [robot_name, qs] : reset_qs)
  {
    auto it = std::find_if(robots.begin(), robots.end(), [&](const auto & r) { return r.name == robot_name; });
    if(it == robots.end())
    {
      continue;
    }
    const auto & rjo = controller->robots().robot(robot_name).module().ref_joint_order();
    auto & qpos_map = per_articulation_qpos[it->prefix];
    for(size_t i = 0; i < it->mj_jnt_names.size(); ++i)
    {
      auto rjo_idx = it->mj_jnt_to_rjo[i];
      if(rjo_idx == -1 || rjo_idx >= static_cast<int>(qs.size()) || it->mj_jnt_names[i].empty())
      {
        continue;
      }
      qpos_map[it->mj_jnt_names[i]] = qs[rjo_idx];
    }
  }
  auto result = urlab->reset("", per_articulation_qpos);
  updateData(result);
  makeDatastoreCalls();
}

bool MjSimImpl::stepSimulation()
{
  if(reset_simulation_)
  {
    resetSimulation(init_qs_, init_pos_);
  }
  auto start_step = clock::now();
  if(config.step_by_step && rem_steps == 0)
  {
    if(controller)
    {
      controller->running = false;
      controller->run();
      controller->running = true;
    }
    mj_sim_start_t = start_step;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return false;
  }
  if(iterCount_ > 0)
  {
    duration_us dt = start_step - mj_sim_start_t;
    mj_sync_delay += duration_us(1e6 * model->opt.timestep) - dt;
    mj_sim_dt[(iterCount_ - 1) % mj_sim_dt.size()] = dt.count();
  }
  mj_sim_start_t = start_step;
  bool done = false;
  if(!config.step_by_step)
  {
    // Note: unlike the original local-stepping implementation, simStep() now blocks on a URLab RPC
    // round-trip rather than a near-instant local mj_step. If that round-trip exceeds the control
    // timestep, sync_real_time's sleep_until below will simply return immediately (no harm done), but
    // it can no longer guarantee tight real-time pacing -- URLab's own physics rate is the actual
    // bottleneck in that case.
    simStep();
  }
  if(config.step_by_step && rem_steps > 0)
  {
    for(size_t i = 0; i < frameskip_; i++)
    {
      simStep();
    }
    rem_steps--;
  }
  if(config.sync_real_time)
  {
    std::this_thread::sleep_until(start_step + duration_us(1e6 * model->opt.timestep) + mj_sync_delay);
  }
  return done;
}

bool MjSimImpl::render()
{
  if(!config.with_mc_rtc_gui || !window)
  {
    return true;
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuiIO & io = ImGui::GetIO();

  if(client)
  {
    client->update();
    client->draw2D(window);
  }
  {
    auto right_margin = 5.0f;
    auto top_margin = 5.0f;
    auto width = io.DisplaySize.x - 2 * right_margin;
    auto height = io.DisplaySize.y - 2 * top_margin;
    ImGui::SetNextWindowPos({0.8f * width - right_margin, top_margin}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({0.2f * width, 0.3f * height}, ImGuiCond_FirstUseEver);
    ImGui::Begin(fmt::format("mc_mujoco (URLab bridge, MuJoCo {})", model ? mj_versionString() : "?").c_str());
    size_t nsamples = std::min(mj_sim_dt.size(), iterCount_);
    mj_sim_dt_average = 0;
    for(size_t i = 0; i < nsamples; ++i)
    {
      mj_sim_dt_average += mj_sim_dt[i] / nsamples;
    }
    ImGui::Text("Average step() round-trip: %.2fus", mj_sim_dt_average);
    ImGui::Text("Simulation/Real time: %.2f", mj_sim_dt_average / (1e6 * model->opt.timestep));
    ImGui::Text("Wallclock time (URLab): %.2fs", wallclock);
    ImGui::Text("URLab endpoint: %s", config.urlab_endpoint.c_str());
    if(ImGui::Checkbox("Sync with real-time", &config.sync_real_time))
    {
      if(config.sync_real_time)
      {
        mj_sync_delay = duration_us(0);
      }
    }
    ImGui::Checkbox("Step-by-step", &config.step_by_step);
    if(config.step_by_step)
    {
      auto doNStepsButton = [&](size_t n, bool final_)
      {
        size_t n_ms = std::ceil(n * 1000 * (controller ? controller->timestep() : model->opt.timestep));
        if(ImGui::Button(fmt::format("+{}ms", n_ms).c_str()))
        {
          rem_steps = n;
        }
        if(!final_)
        {
          ImGui::SameLine();
        }
      };
      doNStepsButton(1, false);
      doNStepsButton(5, false);
      doNStepsButton(10, false);
      doNStepsButton(50, false);
      doNStepsButton(100, true);
    }
    if(ImGui::Button("Reset simulation", ImVec2(-FLT_MIN, 0.0f)))
    {
      reset_simulation_ = true;
    }
    ImGui::End();
  }
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(window);
  glfwPollEvents();

  return !glfwWindowShouldClose(window);
}

void MjSimImpl::stopSimulation() {}

void MjSimImpl::saveGUISettings()
{
  // No 3D camera/visualization state to persist anymore (URLab owns the viewport); kept as a no-op entry
  // point in case future GUI panel settings need persisting.
}

MjSim::MjSim(const MjConfiguration & config) : impl(new MjSimImpl(config))
{
  impl->startSimulation();
}

MjSim::~MjSim()
{
  impl->cleanup();
}

bool MjSim::stepSimulation()
{
  return impl->stepSimulation();
}

void MjSim::resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs,
                            const std::map<std::string, sva::PTransformd> & reset_pos)
{
  impl->resetSimulation(reset_qs, reset_pos);
}

void MjSim::stopSimulation()
{
  impl->stopSimulation();
}

bool MjSim::render()
{
  return impl->render();
}

mc_control::MCGlobalController * MjSim::controller() noexcept
{
  return impl->get_controller();
}

mjModel & MjSim::model() noexcept
{
  return *impl->model;
}

mjData & MjSim::data() noexcept
{
  return *impl->data;
}

} // namespace mc_mujoco
