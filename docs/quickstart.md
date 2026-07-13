# CallmWbcController — Quickstart

Whole-body controller for a **UR5e arm on a TriOrb omnidirectional base** with a
**Robotiq 2F gripper**, coordinated in a single QP. This guide gets you from a fresh
checkout to driving the robot — in RViZ simulation first, then on hardware.

For the design/architecture, see [`../README.md`](../README.md). This doc is the
"how do I run it" companion.

---

## 1. At a glance

| Piece | What it is |
| --- | --- |
| `CallmWbcController` | the controller (this repo) — one QP over arm + base + gripper |
| robots | `ur5e` (0), `triorb` (1), `env/ground` (2), `robotiq_2f_85_gripper` (3) |
| command in | ROS2 `WbcData` message, the GUI, or the datastore |
| base state | SLAM pose (`/robot_pose_slam`) folded into the base joints |
| hardware | `mc_rtde` drives the arm; `TriorbBasePlugin` the base; `RobotiqGripperPlugin` the gripper — **one loop** |

---

## 2. Prerequisites

Built and run inside **`mc_rtc_superbuild`**. The controller needs these sibling
projects available (robot modules + plugins are loaded at runtime, so they're
build-ordering dependencies):

| Dependency | Provides |
| --- | --- |
| [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/) (with the ROS plugin) | framework + `mc_rtc::mc_rtc_ros` |
| `mc_ur5e` / `mc_ur5e_description` | `UR5eFloatingBase` robot module, `Tool` surface |
| `mc_triorb_module` / `mc_triorb_description` | `triorb` robot module |
| `mc_robot_tools` | `Robotiq2f85Gripper` robot module (gripper mesh/collision) |
| `mc_robotiq` | `RobotiqGripperPlugin` (gripper command over socket) |
| `mc_triorb` | `TriorbBasePlugin` (base driver, serial) — hardware only |
| `mc_rtde` | UR hardware interface (`MCControlRtde`) — hardware only |

A working **ROS2** environment is required (the controller runs an `rclcpp` node for
its command/measured topics and the SLAM subscription).

---

## 3. Build

Inside the superbuild, register the project and build. Minimal registration in your
superbuild extension (e.g. `extensions/local.cmake`):

```cmake
AddProject(callm_wbc_controller
  GITHUB_PRIVATE isri-aist/callm_wbc_controller   # or SOURCE_DIR for a local checkout
  DEPENDS mc_rtc mc_ur5e mc_triorb_module mc_robot_tools mc_robotiq
)
```

Then build the superbuild as usual. Standalone (against an installed mc_rtc):

```bash
cd callm_wbc_controller
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
sudo cmake --install build
```

> The build fails fast with a clear message if the mc_rtc ROS plugin
> (`mc_rtc::mc_rtc_ros`) is not available — that target is required.

---

## 4. Run in simulation (RViZ)

Point mc_rtc at the controller. In `~/.config/mc_rtc/mc_rtc.yaml`:

```yaml
MainRobot: UR5eFloatingBase
Enabled: CallmWbcController
Timestep: 0.005
```

Then run the ticker and open RViZ:

```bash
mc_rtc_ticker            # or your preferred mc_rtc simulation front-end
# in another terminal:
roslaunch mc_rtc_ticker display.launch   # RViZ with the mc_rtc panels
```

In sim, **everything hardware-related is inert**: no gripper plugin, no base plugin,
no SLAM. The base is driven by the in-QP velocity integration, and the arm/base/gripper
targets come from the GUI (category **`CallmWbc`**) or the ROS2 client.

The smoke-test config in [`tests/etc/mc_rtc.yaml`](../tests/etc/mc_rtc.yaml) shows a
minimal setup (it additionally enables the gripper plugin).

---

## 5. Run on hardware (arm + base + gripper, one loop)

Use the **`mc_rtde`** interface — it owns the real-time loop and drives the UR arm; the
base and gripper ride along as global plugins. A ready example config is
[`etc/mc_rtde_callm.yaml`](../etc/mc_rtde_callm.yaml):

```bash
MCControlRtde -f /usr/local/etc/mc_controller/etc/mc_rtde_callm.yaml
```

Edit that file for your rig:

- `RTDE.ur5e.ip` — the UR controller IP.
- `TriorbBasePlugin.port` — the base serial device (e.g. `/dev/ttyACM0`).
- `RobotiqGripperPlugin.host` — the UR running the Robotiq URCap socket.

Key points already set for you in that config:

- `MainRobot: UR5eFloatingBase`, `ControlMode: Position`, `Timestep: 0.005` over
  `RobotTimestep: 0.001` (controller must be an integer multiple of the robot loop).
- An `Encoder` observer feeds the UR joint encoders back into the arm.
- `Plugins: [TriorbBasePlugin, RobotiqGripperPlugin]`, with the base plugin set
  `command_in_world_frame: false` (the controller exports a body-frame velocity).

**Data flow on hardware, per tick:**

```
SLAM /robot_pose_slam ─▶ base_x/base_y/base_yaw (control robot)   [before QP]
ROS WbcData ──────────▶ EE / arm posture / base vel / gripper targets
                         │
                         ▼  single QP (arm + base + gripper coordinated)
   arm joint q ─▶ mc_rtde ─▶ UR5e
   base vel   ─▶ Triorb::cmd_velocity ─▶ TriorbBasePlugin ─▶ base
   gripper    ─▶ RobotiqGripper::setOpening ─▶ RobotiqGripperPlugin ─▶ gripper
```

---

## 6. Commanding the robot

Three interchangeable ways; the **datastore is the single source of truth**, and
`run()` just copies the latest command onto the tasks.

### a) GUI (category `CallmWbc`)

- `EE target [world]` — interactive 6-DoF marker for the end-effector.
- `Base velocity [vx, vy, wyaw] (body)` — drive the base by body velocity.
- `Arm posture [rad]` — the 6 UR5e joints.
- `Gripper opening (0=open, 1=closed)` — slider.
- `Task weights [w_ee, w_arm, w_base]` — QP task weights (the "mode": high `w_ee` =
  Cartesian, high `w_arm` = joint-space).
- `Base pose (inactive path)` — absolute base pose (only used with `base_command.mode: pose`).

### b) ROS2 client (`WbcData`)

The controller subscribes to **`callm_wbc/command`** and republishes state on
**`callm_wbc/measured`** (both `std_msgs/Float64MultiArray`, 32 doubles — layout in
[`src/WbcData.h`](../src/WbcData.h)):

| Field | Size | Active | Notes |
| --- | --- | --- | --- |
| `eef_pos` | 3 | ✅ | EE target position, world frame |
| `eef_quat` | 4 | ✅ | EE target orientation `(w,x,y,z)` |
| `posture_arm` | 6 | ✅ | UR5e joints (ref_joint_order) |
| `posture_base` | 3 | ✅ | TriOrb base joints (joint-space base command) |
| `gripper_opening` | 1 | ✅ | **0 = open, 1 = closed** |
| `velocity_base` | 3 | ✅ | `(vx, vy, wyaw)` base body frame |
| `task_weights` | 4 | ✅ | per-task QP weight = **mode**; `<0` keeps default |
| `task_stiffness` | 4 | ✅ | per-task tracking gain = **compliance** |
| `task_damping_ratio` | 4 | ✅ | ζ when stiffness>0 (`D=2ζ√K`); absolute D when stiffness=0 (velocity damper) |

Gain vectors are ordered `[ee, posture_arm, base, base_posture]`.

**Modes = gains (client-side).** Weight the tasks to pick behaviour: high `w_ee` =
Cartesian arm; high `w_posture_arm` with `w_ee=0` = joint-space arm; likewise `w_base`
(velocity) vs `w_base_posture` (joint) for the base. Lower `task_stiffness` for a soft
task. Presets `se3` / `direct` / `joint` / `compliant` live in the Python client
(`WbcData.set_mode("se3")`).

Reference Python client: [`scripts/callm_wbc_client.py`](../scripts/callm_wbc_client.py):

```bash
python3 scripts/callm_wbc_client.py
```

```python
from callm_wbc_client import create_client, WbcData, shutdown_client
import numpy as np

client = create_client()
measured = client.spin_until_measured(timeout_sec=2.0)   # read current state

cmd = WbcData()
cmd.eef_pos  = measured.eef_pos + np.array([0.05, 0.0, 0.05])  # nudge the hand
cmd.eef_quat = measured.eef_quat
cmd.posture_arm = measured.posture_arm
cmd.velocity_base = np.array([0.1, 0.0, 0.0])   # 0.1 m/s forward (body frame)
cmd.gripper_opening = 1.0                        # close
client.send_command(cmd)
shutdown_client(client)
```

> Targets are in the **controller world frame** (fixed where the base spawns at reset).
> A safe pattern is to read `measured`, then command `measured + delta`.

### c) Datastore (in-process, e.g. another plugin)

```cpp
controller.datastore().assign<sva::PTransformd>(CallmWbcController::EE_TARGET_KEY, pose);
controller.datastore().assign<Eigen::Vector3d>(CallmWbcController::BASE_VELOCITY_KEY, {vx, vy, wyaw});
controller.datastore().assign<double>(CallmWbcController::GRIPPER_OPENING_KEY, opening);
```

---

## 7. Base localization (SLAM)

On hardware the base is velocity-controlled, so its **state** is grounded every tick from
a `geometry_msgs/PoseStamped` topic (default `/robot_pose_slam`) — the base analogue of
the arm's encoder feedback. Configured under `base_state` in
[`etc/CallmWbcController.in.yaml`](../etc/CallmWbcController.in.yaml):

```yaml
base_state:
  from_slam: true
  slam_topic: "/robot_pose_slam"
  slam_frame: world          # "world" (raw) or "capture_offset"
  slam_timeout: 0.5          # s; hold-last + warn once beyond this
  command_key: "Triorb::cmd_velocity"
```

- `slam_frame: world` — use the SLAM pose raw. Valid when your SLAM **zeroes its origin
  to the robot at each episode start** (so its `map` frame == the controller world frame).
- `slam_frame: capture_offset` — grab the first post-reset SLAM pose as the `map→world`
  offset. Robust to any SLAM map origin; use this if SLAM is *not* re-zeroed per episode.

Sanity check for `world` mode: right after a reset, `/robot_pose_slam` should read
≈ `(0, 0, 0)` with identity orientation (position **and** heading zeroed).

---

## 8. Configuration cheat-sheet

| File | Purpose |
| --- | --- |
| [`etc/CallmWbcController.in.yaml`](../etc/CallmWbcController.in.yaml) | controller config: task gains, gripper, ROS topics, `base_state` |
| [`etc/mc_rtde_callm.yaml`](../etc/mc_rtde_callm.yaml) | example **hardware** run config (mc_rtde + both plugins) |
| [`tests/etc/mc_rtc.yaml`](../tests/etc/mc_rtc.yaml) | smoke-test / minimal run config |

Notable controller-config knobs:

```yaml
ee_task:      { stiffness: 5.0,  weight: 1000.0 }   # arm end-effector
base_task:    { stiffness: 2.0,  weight: 1000.0 }   # base transform (+ optional `damping`)
ur5e_posture: { stiffness: 10.0, weight: 5.0 }      # arm redundancy
base_command: { mode: velocity }                    # "velocity" (active) or "pose"
gripper:      { enable: true, model: Robotiq2f85Gripper }
```

> **Damping note:** there is no generic "damping task" in mc_rtc
> (`mc_tasks::force::DampingTask` is a force/admittance task needing a force sensor).
> Base damping = the `base_task.damping` gain (default `2*sqrt(stiffness)`).

---

## 9. Troubleshooting

| Symptom | Likely cause / fix |
| --- | --- |
| `mc_rtc_ros is required` at build | mc_rtc built without the ROS plugin; rebuild it with ROS support |
| Warning: *actuated robot not configured: triorb / robotiq...* (mc_rtde) | expected — mc_rtde only drives the UR; the base/gripper are driven by their plugins |
| Base doesn't move on hardware | `TriorbBasePlugin` not enabled, wrong serial `port`, or `Triorb::cmd_velocity` not being written (check the plugin is in `Plugins:`) |
| Gripper doesn't respond | `RobotiqGripperPlugin` not enabled, or the Robotiq URCap socket (`host:63352`) unreachable |
| Arm flies off toward a bad pose | EE target is **world-frame absolute**; command `measured + delta`, not a raw guess |
| Base pose drifts / EE misses in the room | SLAM not aligned — check `slam_frame` and that SLAM reads `(0,0,0)` after reset |
| `WbcData size mismatch: expected 20` | client sent the wrong-length array; match the layout in `WbcData.h` |
| Controller can't keep up at 1 kHz | run the controller slower than the robot loop (`Timestep` a multiple of `RobotTimestep`, e.g. 0.005 / 0.001) |

---

## See also

- [`../README.md`](../README.md) — architecture, task set, command contract, design rationale.
- [`src/WbcData.h`](../src/WbcData.h) — the exact command message layout.
- [`scripts/callm_wbc_client.py`](../scripts/callm_wbc_client.py) — reference ROS2 client.
