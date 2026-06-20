# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

mc_mujoco is a bridge between mc_rtc (a robotics control framework) and
[URLab](https://github.com/URLab-Sim/UnrealRoboticsLab), which runs MuJoCo physics inside Unreal Engine.
mc_mujoco does **not** run a local MuJoCo simulation or render a 3D scene: it connects to URLab's ZMQ RPC
bridge, retrieves the compiled MuJoCo model once (for kinematic/sensor bookkeeping only), computes
control with mc_rtc each tick, and exchanges state/commands with URLab over the network. Physics
stepping and 3D rendering happen entirely on the URLab/Unreal side. mc_mujoco still shows a small mc_rtc
GUI window (2D ImGui panel only — forms, plots, controller tree) alongside the Unreal window.

## Development Environment

**All building, testing, and debugging MUST be done inside the Docker container.** Do not attempt to build or run locally — the host may lack required dependencies (mc_rtc, MuJoCo, ROS 2, etc.).

```bash
# Build and get a dev shell (builds mc_mujoco, then drops to bash inside container)
cd docker && make run

# Run CI tests (build + standalone library test)
cd docker && make ci-test
```

Inside the container:
- Source is mounted at `/workspace/mc_mujoco`
- Build dir: `/build` — backed by Docker volume `mc_mujoco_build_<MUJOCO_VERSION>`.
  Volume name encodes the `MUJOCO_VERSION` pin so a version bump automatically
  points at a fresh cache; `make` prunes stale-version volumes on each run.
- MuJoCo at `/opt/mujoco`
- Rebuild: `cmake --build /build`
- Reinstall: `cmake --install /build`
- Run standalone tests: `ctest -V --test-dir /ci-build-standalone`

To run arbitrary commands in the dev container:
```bash
docker compose -f docker/docker-compose.yml run --rm mc-rtc-mujoco bash -c "<command>"
```

**CMake options:** `USE_GL` (default ON) enables OpenGL rendering; `MUJOCO_ROOT_DIR` is set to `/opt/mujoco` in the container.

**Submodules:** Clone with `--recursive` (mc_rtc-imgui for the GUI panel).

## Formatting & Linting

- `.clang-format`: Allman style, 120 column limit, 2-space indent
- `pre-commit run --all-files` runs clang-format, cmake-format, and standard checks
- `ext/` directory is excluded from formatting

## Architecture

### Core flow (src/main.cpp)
1. Parses CLI args (URLab endpoint/timeouts, config file, torque control, step-by-step, GUI flags)
2. Creates `MjSim`, which connects to URLab (`hello`, `begin_pie` if needed), loads the compiled model
   (`mjb`) it returns, and matches mc_rtc robots to URLab articulations by name
3. Spawns a separate simulation thread (`stepSimulation` loop, sends `step` RPCs to URLab) and runs the
   GUI render loop (2D ImGui panel only) on the main thread

### Key source files
- **mj_sim.h/cpp** — Public simulator API: `stepSimulation()`, `render()`, `resetSimulation()`, access to `MCGlobalController`, the local MuJoCo model/data mirror
- **mj_sim_impl.h** — Internal state: `MjRobot` (name-keyed bridge between an mc_rtc robot and a URLab articulation — no longer indexes a locally-stepped `mjData` by integer id), `MjSimImpl` (owns the `URLabClient`, the local kinematic mirror, the controller, the GUI)
- **mj_utils.cpp** — URLab handshake (`hello`/`begin_pie`), loading the compiled model from the handshake's `mjb` via a temp-file round-trip into `mj_loadModel`, matching mc_rtc robots to URLab articulations, GUI window creation (2D only)
- **urlab_client.h/cpp** — Standalone ZMQ REQ/REP + msgpack RPC client implementing URLab's wire protocol (`hello`, `begin_pie`, `step`, `reset`). This is the only place that talks to the network; everything else works with the parsed `URLabHandshake`/`URLabStepResult` structs it returns
- **MujocoClient.h/cpp** — GUI client extending `mc_rtc::imgui::Client`, 2D ImGui panel only (no 3D scene is rendered locally, so all 3D-space GUI element overrides — point3d, force, arrow, polygon, polyhedron, etc. — are no-ops)
- **mj_configuration.h** — Configuration struct (URLab endpoint/timeouts, controller, GUI, torque control, real-time sync flags)

Physics is never stepped locally (`mj_step` is never called). The local `mjModel`/`mjData` loaded from
URLab's `mjb` exist purely so mc_rtc can read mj-derived quantities the way it always has: each
`step`/`reset` reply's qpos/qvel are written into the local `mjData`, then `mj_forward` (not `mj_step`)
recomputes everything derived (sensor readings, `qfrc_actuator`, body transforms) without integrating.

### mc_rtc framework concepts

mc_rtc is a unified interface for simulation and robot control. Key concepts relevant to this project:

- **MCGlobalController** — Top-level controller manager. Handles sensor data, loads controllers dynamically, manages plugins and observers. mc_mujoco instantiates this to run mc_rtc controllers, with sensor input sourced from URLab instead of local physics.
- **MCController** — Base class for all controllers. Manages robots, tasks, constraints, QPSolver, datastore, logger, and GUI. Assumes at least 2 robots (first is "main").
- **FSM Controller** — Inherits MCController. Manages state machines for complex robot behaviors with configurable transitions.
- **States** — FSM building blocks with lifecycle: construct → init → run (until done) → teardown → destruct. Loaded dynamically from shared libraries via `StateFactory`. Registered with `EXPORT_SINGLE_STATE("Name", Type)`.
- **GlobalPlugin** — Plugins loaded by MCGlobalController with `init()`, `reset()`, `before()`, `after()` hooks. Registered with `EXPORT_MC_RTC_PLUGIN("Name", Type)`.
- **Observers** — State estimators (encoders, IMU fusion) with `run()` before controller and `update()` to write estimates. Registered with `EXPORT_OBSERVER_MODULE("Name", Type)`.
- **Datastore** — Key-value store shared between controller and interface. mc_mujoco exposes callbacks (`SetPDGains`, `GetPDGains`, etc.) through the datastore. `SetPosW` is a no-op in this bridge: robot/object poses are authoritative in URLab.
- **Configuration** — mc_rtc reads config from `$HOME/.config/mc_rtc/mc_rtc.conf` (user) and per-controller/plugin YAML files.

### Model and scene authoring
Model geometry, scene composition, and object placement are no longer mc_mujoco's responsibility — they
are authored in the Unreal/URLab level and compiled by URLab, which sends the compiled model (`mjb`) to
mc_mujoco at handshake time. Per-robot PD gains are still configured locally via `<robot>.yaml`
(`pdGainsPath`), searched under `~/.config/mc_rtc/mc_mujoco/` then the share install directory.

### Vendored libraries (ext/)
imgui, implot, pugixml, glfw (conditional), mc_rtc-imgui. Built as part of the `mc_mujoco_lib` static
library. `libzmq`/`cppzmq`/`msgpack-c` (the URLab bridge transport/encoding) are found as system packages
or fetched from source by CMake if unavailable.
