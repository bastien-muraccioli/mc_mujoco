#include "glfw3.h"
#include "mujoco.h"

#include "mj_utils.h"

#include "config.h"

#include "imgui.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot.h"

#include "Robot_Regular_ttf.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
namespace fs = std::filesystem;

namespace mc_mujoco
{

/*******************************************************************************
 * Global library state
 ******************************************************************************/

static bool glfw_initialized = false;

/*******************************************************************************
 * Callbacks for GLFWwindow (mc_rtc GUI panel only -- no 3D scene to interact with)
 ******************************************************************************/

namespace
{

void glfwKeyCallback(GLFWwindow * window, int key, int /*scancode*/, int action, int /*mods*/)
{
  auto mj_sim = static_cast<MjSimImpl *>(glfwGetWindowUserPointer(window));
  if(!mj_sim || action != GLFW_PRESS)
  {
    return;
  }
  if(ImGui::GetIO().WantCaptureKeyboard)
  {
    return;
  }
  if(key == GLFW_KEY_SPACE)
  {
    mj_sim->config.step_by_step = !mj_sim->config.step_by_step;
  }
  if(key == GLFW_KEY_RIGHT && mj_sim->config.step_by_step)
  {
    mj_sim->rem_steps = 1;
  }
}

/** Write a binary blob to a temp file and return the path. Used to round-trip URLab's compiled mjb
 * through mj_loadModel, which only reads from disk. */
std::string write_temp_mjb(const std::string & mjb)
{
  auto path =
      fs::temp_directory_path()
      / fmt::format("mc_mujoco_urlab_{}.mjb", std::chrono::high_resolution_clock::now().time_since_epoch().count());
  std::ofstream out(path, std::ios::binary);
  if(!out.is_open())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Failed to open temp file {} to write URLab model",
                                                     path.string());
  }
  out.write(mjb.data(), static_cast<std::streamsize>(mjb.size()));
  out.close();
  return path.string();
}

/** Resolve a robot's reference-joint-order joint names and actuator names into the corresponding live
 * MuJoCo names for a given articulation, using the handshake's original_names map (authored XML name ->
 * live/possibly-renamed name) when available, falling back to "<prefix>_<name>" (mc_mujoco's historical
 * naming convention, still used by mc_rtc robot modules' MJCF) otherwise. */
struct ResolvedNames
{
  std::vector<std::string> joint_names; // live MuJoCo joint name per rjo entry, "" if not found
  std::vector<std::string> actuator_names; // live MuJoCo actuator name per rjo entry, "" if no actuator
};

ResolvedNames resolve_names(const mjModel & model,
                            const URLabArticulationInfo & art,
                            const std::vector<std::string> & rjo)
{
  ResolvedNames out;
  out.joint_names.resize(rjo.size());
  out.actuator_names.resize(rjo.size());

  auto joints_it = art.original_names.find("joints");
  auto actuators_it = art.original_names.find("actuators");

  // Build reverse maps: authored XML name -> live name (the handshake gives live -> original; we want
  // the other direction to look up by the robot module's ref_joint_order, which uses authored names).
  std::unordered_map<std::string, std::string> orig_to_live_joint;
  if(joints_it != art.original_names.end())
  {
    for(const auto & [live, orig] : joints_it->second)
    {
      orig_to_live_joint[orig] = live;
    }
  }
  std::unordered_map<std::string, std::string> orig_to_live_actuator;
  if(actuators_it != art.original_names.end())
  {
    for(const auto & [live, orig] : actuators_it->second)
    {
      orig_to_live_actuator[orig] = live;
    }
  }

  for(size_t i = 0; i < rjo.size(); ++i)
  {
    const auto & jn = rjo[i];

    std::string live_joint;
    if(auto it = orig_to_live_joint.find(jn); it != orig_to_live_joint.end())
    {
      live_joint = it->second;
    }
    else
    {
      // No rename reported (or original_names omitted entirely): fall back to the historical mc_mujoco
      // convention of "<prefix>_<name>".
      live_joint = art.prefix.empty() ? jn : fmt::format("{}_{}", art.prefix, jn);
    }
    if(mj_name2id(&model, mjOBJ_JOINT, live_joint.c_str()) != -1)
    {
      out.joint_names[i] = live_joint;
    }

    std::string live_act;
    if(auto it = orig_to_live_actuator.find(jn); it != orig_to_live_actuator.end())
    {
      live_act = it->second;
    }
    else
    {
      live_act = art.prefix.empty() ? jn : fmt::format("{}_{}", art.prefix, jn);
    }
    if(mj_name2id(&model, mjOBJ_ACTUATOR, live_act.c_str()) != -1)
    {
      out.actuator_names[i] = live_act;
    }
  }

  return out;
}

} // namespace

/*******************************************************************************
 * URLab handshake / model loading
 ******************************************************************************/

bool mujoco_init(MjSimImpl * mj_sim)
{
  mc_rtc::log::info("[mc_mujoco] Connecting to URLab at {}", mj_sim->config.urlab_endpoint);
  URLabHandshake hs;
  try
  {
    hs = mj_sim->urlab->hello();
    if(!hs.manager_present)
    {
      mc_rtc::log::info("[mc_mujoco] URLab editor has no active PIE session, requesting begin_pie...");
      hs = mj_sim->urlab->beginPIE(mj_sim->config.urlab_pie_timeout_s);
    }
  }
  catch(const URLabError & e)
  {
    mc_rtc::log::error("[mc_mujoco] URLab handshake failed: {}", e.what());
    return false;
  }

  if(hs.mjb.empty())
  {
    mc_rtc::log::error("[mc_mujoco] URLab handshake did not provide a compiled model (mjb)");
    return false;
  }

  mc_rtc::log::info("[mc_mujoco] Connected to URLab {} (MuJoCo {}), session {}", hs.urlab_version, hs.mujoco_version,
                    hs.session_id);

  // mj_loadModel only reads from disk; round-trip the compiled binary through a temp file.
  auto tmp_path = write_temp_mjb(hs.mjb);
  char error[1000] = "Could not load model from URLab";
  mj_sim->model = mj_loadModel(tmp_path.c_str(), nullptr, error, 1000);
  std::remove(tmp_path.c_str());
  if(!mj_sim->model)
  {
    mc_rtc::log::error("[mc_mujoco] Failed to load URLab-provided model: {}", error);
    return false;
  }
  mj_sim->data = mj_makeData(mj_sim->model);

  // Match every mc_rtc robot to a URLab articulation by prefix == robot module name. This mirrors the
  // historical mc_mujoco convention where the MJCF model name equals the mc_rtc robot module name.
  for(const auto & r : mj_sim->get_controller()->robots())
  {
    auto art_it = std::find_if(hs.articulations.begin(), hs.articulations.end(),
                               [&](const auto & a) { return a.prefix == r.module().name; });
    if(art_it == hs.articulations.end())
    {
      // Not every mc_rtc robot needs a URLab articulation (e.g. a purely kinematic "robot" used for
      // planning only); skip silently, as the original local-mujoco code allowed this too (objects vs.
      // robots note in the README).
      continue;
    }

    MjRobot robot;
    robot.name = r.name();
    robot.prefix = art_it->prefix;

    const auto & rjo = r.module().ref_joint_order();
    auto resolved = resolve_names(*mj_sim->model, *art_it, rjo);
    robot.mj_jnt_names = resolved.joint_names;
    robot.mj_act_names = resolved.actuator_names;

    // Root body / joint: derive from the robot's mb() root link, prefixed the same way joints are.
    if(r.mb().nrJoints() > 0 && r.mb().joint(0).type() == rbd::Joint::Type::Free)
    {
      robot.root_joint_type = mjJNT_FREE;
      robot.root_body =
          art_it->prefix.empty() ? r.mb().body(0).name() : fmt::format("{}_{}", art_it->prefix, r.mb().body(0).name());
      // The free joint itself does not have a stable authored name we can resolve through
      // original_names (it is typically anonymous in the MJCF); fall back to the body name suffixed
      // with "_freejoint", matching mc_mujoco's historical robot-module convention. If this does not
      // resolve, root_joint_type detection below (via the body's first joint) is used instead.
      auto candidate = fmt::format("{}_freejoint", robot.root_body);
      if(mj_name2id(mj_sim->model, mjOBJ_JOINT, candidate.c_str()) != -1)
      {
        robot.root_joint = candidate;
      }
    }

    mj_sim->robots.push_back(std::move(robot));
  }

  if(mj_sim->robots.empty())
  {
    mc_rtc::log::warning("[mc_mujoco] No mc_rtc robot matched a URLab articulation by prefix. Check that each robot "
                         "module's name matches the MJCF model name URLab reports.");
  }

  return true;
}

/*******************************************************************************
 * GUI window (mc_rtc 2D panel only)
 ******************************************************************************/

void mujoco_create_window(MjSimImpl * mj_sim)
{
  if(!glfw_initialized)
  {
    if(!glfwInit())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] GLFW initialization failed");
    }
    glfw_initialized = true;
  }

  // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
  const char * glsl_version = "#version 100";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
  const char * glsl_version = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
  const char * glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

  // Small, fixed-size window: this is a GUI panel (mc_rtc forms/plots/tree), not a 3D viewport, so it
  // does not need to be large. Resizable in case panels grow.
  mj_sim->window =
      glfwCreateWindow(900, 700, "mc_mujoco (mc_rtc GUI -- robot rendered in Unreal Engine)", nullptr, nullptr);
  if(!mj_sim->window)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] GLFW window creation failed");
  }
  glfwMakeContextCurrent(mj_sim->window);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(mj_sim->window, static_cast<void *>(mj_sim));
  glfwSetKeyCallback(mj_sim->window, glfwKeyCallback);

  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO & io = ImGui::GetIO();
  ImFontConfig fontConfig;
  fontConfig.FontDataOwnedByAtlas = false;
  ImVector<ImWchar> ranges;
  ImFontGlyphRangesBuilder builder;
  builder.AddText(u8"\u03bc");
  builder.AddRanges(io.Fonts->GetGlyphRangesDefault());
  builder.BuildRanges(&ranges);
  io.FontDefault =
      io.Fonts->AddFontFromMemoryTTF(Roboto_Regular_ttf, Roboto_Regular_ttf_len, 18.0f, &fontConfig, ranges.Data);
  io.Fonts->Build();
  io.IniFilename = "/tmp/imgui.ini";

  ImGui::StyleColorsLight();
  auto & style = ImGui::GetStyle();
  style.FrameRounding = 6.0f;
  auto & bgColor = style.Colors[ImGuiCol_WindowBg];
  bgColor.w = 0.95f;

  ImGui_ImplGlfw_InitForOpenGL(mj_sim->window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
}

void mujoco_cleanup(MjSimImpl * mj_sim)
{
  if(mj_sim->window)
  {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(mj_sim->window);
  }

  mj_deleteData(mj_sim->data);
  mj_deleteModel(mj_sim->model);

  // FIXME glfwTerminate will segfault so we never de-init glfw
  // Ref: http://www.mujoco.org/forum/index.php?threads/segmentation-fault-for-record-on-ubuntu-16-04.3516/#post-4205
  // glfw_initialized = false;
  // glfwTerminate();
}

int mujoco_get_sensor_id(const mjModel & m, const std::string & name, mjtSensor type)
{
  auto id = mj_name2id(&m, mjOBJ_SENSOR, name.c_str());
  return (id != -1 && m.sensor_type[id] == type) ? id : -1;
}

void mujoco_get_sensordata(const mjModel & model, const mjData & data, int sensor_id, double * sensor_reading)
{
  if(sensor_id == -1)
  {
    return;
  }
  std::memcpy(sensor_reading, &data.sensordata[model.sensor_adr[sensor_id]],
              model.sensor_dim[sensor_id] * sizeof(double));
}

} // namespace mc_mujoco
