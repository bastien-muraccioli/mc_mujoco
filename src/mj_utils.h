#ifndef MJ_UTILS_H
#define MJ_UTILS_H

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <vector>

#include "mj_sim_impl.h"

namespace mc_mujoco
{

/*! Connect to URLab, perform the handshake (+ begin_pie if needed), load the resulting compiled model
 * locally, and populate mj_sim->robots / mj_sim->objects from the handshake's articulations block
 * matched against the mc_rtc controller's robots. */
bool mujoco_init(MjSimImpl * mj_sim);

/*! Create the GLFW window used for the mc_rtc GUI panel (2D ImGui only) */
void mujoco_create_window(MjSimImpl * mj_sim);

/** Returns a sensor id from name, -1 if the type does not match or the sensor does not exist */
int mujoco_get_sensor_id(const mjModel & m, const std::string & name, mjtSensor type);

/** Reads a MuJoCo sensor into the provided data pointer, the data must have the correct size for the sensor type */
void mujoco_get_sensordata(const mjModel & model, const mjData & data, int sensor_id, double * sensor_reading);

/*! Cleanup. */
void mujoco_cleanup(MjSimImpl * mj_sim);

} // namespace mc_mujoco

#endif // MJ_UTILS_H_
