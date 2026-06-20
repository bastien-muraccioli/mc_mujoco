#include "MujocoClient.h"

namespace mc_mujoco
{

void MujocoClient::draw2D(GLFWwindow * window)
{
  int width;
  int height;
  glfwGetWindowSize(window, &width, &height);

  mc_rtc::imgui::Client::draw2D({static_cast<float>(width), static_cast<float>(height)});
}

} // namespace mc_mujoco
