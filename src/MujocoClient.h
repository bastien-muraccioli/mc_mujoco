#pragma once

#include "Client.h"

#include "glfw3.h"
#include "imgui.h"

namespace mc_mujoco
{

using Client = mc_rtc::imgui::Client;
using ElementId = mc_rtc::imgui::ElementId;

/** mc_rtc ControllerClient: 2D ImGui panel only.
 *
 * URLab renders the robot and the 3D scene in Unreal Engine; mc_mujoco no longer renders a MuJoCo 3D
 * viewport, so all 3D-space GUI elements (ImGuizmo overlays, force/arrow/polygon/polyhedron markers
 * drawn into a 3D view) have nothing to draw into and are implemented as no-ops below. The 2D panel
 * (forms, buttons, plots, tree view -- i.e. mc_rtc's actual interactive GUI) is unaffected and works
 * exactly as before.
 */
struct MujocoClient : public mc_rtc::imgui::Client
{
  using mc_rtc::imgui::Client::Client;

  void draw2D(GLFWwindow * window);

protected:
  void point3d(const ElementId &,
               const ElementId &,
               bool,
               const Eigen::Vector3d &,
               const mc_rtc::gui::PointConfig &) override
  {
  }

  void rotation(const ElementId &, const ElementId &, bool, const sva::PTransformd &) override {}

  void transform(const ElementId &, const ElementId &, bool, const sva::PTransformd &) override {}

  void xytheta(const ElementId &, const ElementId &, bool, const Eigen::Vector3d &, double) override {}

  void polygon(const ElementId &,
               const std::vector<std::vector<Eigen::Vector3d>> &,
               const mc_rtc::gui::LineConfig &) override
  {
  }

  inline void polygon(const ElementId & id,
                      const std::vector<std::vector<Eigen::Vector3d>> & points,
                      const mc_rtc::gui::Color & color) override
  {
    polygon(id, points, mc_rtc::gui::LineConfig(color));
  }

  void polyhedron(const ElementId &,
                  const std::vector<std::array<Eigen::Vector3d, 3>> &,
                  const std::vector<std::array<mc_rtc::gui::Color, 3>> &,
                  const mc_rtc::gui::PolyhedronConfig &) override
  {
  }

  void force(const ElementId &,
             const ElementId &,
             const sva::ForceVecd &,
             const sva::PTransformd &,
             const mc_rtc::gui::ForceConfig &,
             bool) override
  {
  }

  void arrow(const ElementId &,
             const ElementId &,
             const Eigen::Vector3d &,
             const Eigen::Vector3d &,
             const mc_rtc::gui::ArrowConfig &,
             bool) override
  {
  }

  void trajectory(const ElementId &, const std::vector<Eigen::Vector3d> &, const mc_rtc::gui::LineConfig &) override {}

  void trajectory(const ElementId &, const std::vector<sva::PTransformd> &, const mc_rtc::gui::LineConfig &) override {}

  void trajectory(const ElementId &, const Eigen::Vector3d &, const mc_rtc::gui::LineConfig &) override {}

  void trajectory(const ElementId &, const sva::PTransformd &, const mc_rtc::gui::LineConfig &) override {}

  void visual(const ElementId &, const rbd::parsers::Visual &, const sva::PTransformd &) override {}

  mc_rtc::imgui::InteractiveMarkerPtr make_marker(const sva::PTransformd &, mc_rtc::imgui::ControlAxis) override
  {
    return {};
  }
};

} // namespace mc_mujoco
