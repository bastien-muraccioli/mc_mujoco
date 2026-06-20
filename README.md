# mc_mujoco — URLab bridge for mc-rtc

`mc_mujoco` runs an [mc_rtc](https://github.com/jrl-umi3218/mc_rtc) controller against physics simulated
by [MuJoCo](https://mujoco.org/) inside [Unreal Engine](https://www.unrealengine.com/), via
[URLab](https://github.com/URLab-Sim/UnrealRoboticsLab). mc_mujoco does not run a local MuJoCo simulation
and does not render a 3D scene: physics is stepped and rendered entirely on the URLab/Unreal side.
mc_mujoco connects to URLab's ZMQ RPC bridge, retrieves the compiled MuJoCo model, computes control with
mc_rtc, and sends actuator commands back every physics step. A small mc_rtc GUI window (2D ImGui panel:
forms, plots, the controller tree) runs alongside the Unreal window.

```
┌──────────────────────────┐        ZMQ RPC (msgpack)        ┌───────────────────────────┐
│  Unreal Engine + URLab    │ <------------------------------> │  mc_mujoco                 │
│  - steps MuJoCo physics   │   hello / step / reset           │  - mc_rtc controller       │
│  - renders robot + scene  │   per-articulation qpos/qvel/    │  - PD or torque control    │
│                            │   sensors out, ctrl_map in       │  - mc_rtc GUI (2D panel)   │
└──────────────────────────┘                                   └───────────────────────────┘
```

## Requirements

- A running URLab session (Unreal Editor with the URLab plugin, in Play-In-Editor or ready to enter it)
  exposing its ZMQ RPC bridge — see [URLab-Sim/UnrealRoboticsLab](https://github.com/URLab-Sim/UnrealRoboticsLab).
- Each mc_rtc robot you intend to control must correspond to a URLab articulation whose **prefix matches
  the mc_rtc robot module name** (mc_mujoco's historical MJCF-naming convention). PD gains are still
  configured locally — see [Configuration](#configuration) below — the model geometry itself is no
  longer read from disk by mc_mujoco.
- Linux only.

## Usage

Install the required apt packages:

```sh
$ sudo apt install libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libglew-dev libzmq3-dev
```

Then build `mc_mujoco` (this also downloads MuJoCo to `$HOME/.mujoco/mujoco<version>` if
`MUJOCO_ROOT_DIR` isn't provided — the local MuJoCo install is used only to load the model URLab provides
and recompute kinematics, never to step physics):

```sh
$ git clone --recursive git@github.com:rohanpsingh/mc_mujoco.git
$ cd mc_mujoco
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
$ make
$ make install
```

`libzmq`/`cppzmq`/`msgpack-c` are fetched from source automatically if not found as system packages (see
`src/CMakeLists.txt`); install `libzmq3-dev`, `cppzmq-dev`, and `libmsgpack-cxx-dev` beforehand to skip
that.

Add the following line to your `~/.bashrc` if MuJoCo was auto-downloaded:

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${HOME}/.mujoco/mujoco<version>/lib:${HOME}/.mujoco/mujoco<version>/bin
```

With a URLab session running (or ready to enter Play-In-Editor) and listening on its ZMQ bridge, run:

```sh
$ mc_mujoco --urlab-endpoint tcp://localhost:5559
```

### CLI options

| Flag                       | Description                                                                 |
| --------------------------- | ---------------------------------------------------------------------------- |
| `-f, --mc-config`           | Configuration file passed to mc_rtc                                          |
| `--urlab-endpoint`          | URLab ZMQ RPC endpoint (default `tcp://localhost:5559`)                      |
| `--urlab-timeout-ms`        | Timeout for a single URLab RPC round-trip (default 6000)                     |
| `--urlab-pie-timeout-s`     | Timeout waiting for URLab to enter Play-In-Editor if not already running     |
| `--step-by-step`            | Start paused, advance manually from the GUI                                  |
| `--torque-control`          | Send raw torque commands instead of running mc_mujoco's local PD loop        |
| `-s, --sync`                | Pace `step()` RPCs against wall-clock real time                              |
| `--without-controller`      | Disable the mc_rtc controller (bridge connects but sends no control)         |
| `--without-mc-rtc-gui`      | Disable the mc_rtc GUI window                                                |

## Docker

Assuming [Docker](https://docs.docker.com/engine/install/) (Compose v2) has been installed:

```sh
$ git clone --recursive git@github.com:rohanpsingh/mc_mujoco.git
$ cd mc_mujoco/docker
$ make run
```

This builds the environment image, compiles mc_mujoco, and drops you into a shell. Then:

```sh
$ mc_mujoco --urlab-endpoint tcp://<urlab-host>:5559
```

See [docker/README.md](docker/README.md) for details on development workflow and version management.

## Configuration

PD gains are still configured the way they always were: each robot's `<robot>.yaml` (searched under
`~/.config/mc_rtc/mc_mujoco/` then the share install directory) may set `pdGainsPath` to a gains file.
Model geometry, scene composition, and object placement are no longer configured here — they're authored
in the Unreal/URLab level itself.

## Datastore callbacks

The following callbacks are available to the controller when running inside `mc_mujoco`:

| Signature                                                                            | Description                                                                                                        |
| ------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `{robot}::SetPDGains(const std::vector<double> & p, const std::vector<double> & d)`  | Set the PD gains for the actuation of the given `robot`. `p` and `d` must follow the robot's reference joint order |
| `{robot}::SetPDGainsByName(const std::string & jn, double p, double d)`              | Set the PD gains for a given joint `jn` in `robot`                                                                 |
| `{robot}::GetPDGains(std::vector<double> & p, std::vector<double> & d)`              | Get the current PD gains for `robot` actuation                                                                     |
| `{robot}::GetPDGainsByName(const std::string & jn, double & p, double & d)`          | Get the current PD gains for a given joint `jn` in `robot`                                                         |
| `{robot}::SetPosW(const sva::PTransformd &)`                                          | **No-op.** Robot/object poses are authoritative in URLab; logs a one-time warning if called.                       |

## Citation

```
@inproceedings{singh2023mc,
  title={mc-mujoco: Simulating Articulated Robots with FSM Controllers in MuJoCo},
  author={Singh, Rohan P and Gergondet, Pierre and Kanehiro, Fumio},
  booktitle={2023 IEEE/SICE International Symposium on System Integration (SII)},
  pages={1--5},
  year={2023},
  organization={IEEE}
}
```

### Credits

This package includes code from:

- [imgui v1.84.2](https://github.com/ocornut/imgui/)
- [implot v0.11](https://github.com/epezent/implot)
- [pugixml v1.11.4](https://github.com/zeux/pugixml)
- [Roboto font](https://github.com/googlefonts/roboto)

Physics simulation and rendering are provided by [URLab](https://github.com/URLab-Sim/UnrealRoboticsLab),
not by this package.
