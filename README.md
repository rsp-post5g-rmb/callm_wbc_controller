CallmWbcController
==

Whole-body controller ([mc_rtc]) that coordinates a **UR5e arm** mounted on a
**TriOrb omnidirectional base**, with a **Robotiq 2F gripper** on the tool, in a
single QP. It is adapted from the mc_rtc *Mobile arm controller* tutorial,
replacing Dingo with the TriOrb module and restructuring it as a reusable WBC
controller (no scripted phase machine) driven by a ROS2 command client.

> **New here?** Start with [`docs/quickstart.md`](docs/quickstart.md) — build, run in
> RViZ, run on hardware, and command the robot. This README covers the design.

Model
--

> **Branch `no_gripper_sim`:** the gripper **model** is not loaded/simulated
> (`gripper.simulate: false`) — so this build has no dependency on the
> `mc_robot_tools` gripper module — but the **real** gripper is still driven via the
> `mc_robotiq` plugin (`gripper.command: true`). See the gripper notes below.

| Index | Robot | Role |
| --- | --- | --- |
| 0 | `UR5eFloatingBase` | main robot, arm with a floating base |
| 1 | `triorb` | the mobile base |
| 2 | `env/ground` | visual reference only |
| (3) | `robotiq_2f_85_gripper` | gripper model — **only when `gripper.simulate: true`** (off on this branch) |

- The UR5e floating base is rigidly attached to the TriOrb `mount` link through a
  Base&harr;Base contact. When the QP moves the base the arm follows; when the
  end-effector task pulls the hand, the solver distributes motion across both the
  arm joints and the base dof &mdash; whole-body coordination with no explicit IK.
- **No base&harr;ground dof-masked contact is used.** The TriOrb module is
  *fixed-base* with explicit planar joints (`base_x`, `base_y`, `base_yaw`), so
  planar motion is intrinsic to the kinematics and solved directly by the QP.
- The TriOrb description ships no RSDF surfaces, so the attachment surface
  `ArmMount` is declared on the `mount` link at runtime, in the constructor.
- The **Robotiq gripper** has two independent switches:
  - `gripper.simulate` — load the gripper **model** (a `mc_robot_tools`
    `ConnectableRobotModule`, `Robotiq2f85Gripper` / `Robotiq2f140Gripper`) as an
    extra robot and bolt it onto the UR5e `Tool` surface via a rigid
    `Tool`&harr;`Base` contact (`defaultMountingTransform` = `RotZ(pi)`). **Off on
    this branch**, so there is no `mc_robot_tools` dependency.
  - `gripper.command` — forward `gripper_opening` to the `mc_robotiq` plugin
    (`RobotiqGripper::setOpening`) to drive the **real** gripper. **On.** This path
    is independent of the model: it only needs the plugin's datastore call to exist.

Tasks / constraints
--

- `contactConstraint`, `kinematicsConstraint` (UR5e), `selfCollisionConstraint`
  (UR5e), plus a per-robot `KinematicsConstraint` for the TriOrb (and one for the
  gripper only when `gripper.simulate: true`).
- **Arm&harr;base collision avoidance** via `addCollisions("ur5e", "triorb", ...)`.
  mc_rtc auto-builds an `sch::S_Box` collision convex (named `base`) from the
  TriOrb URDF `<box>`, so no hull file is needed. Guarded links and distances are
  set in `arm_base_collision`; `base_link`/`shoulder_link` are excluded (the arm
  base is mounted on the base).
- `SurfaceTransformTask` on the UR5e `Tool` surface (arm end-effector).
- `TransformTask` on the TriOrb `base` body (base command).
- `PostureTask` for the UR5e (arm redundancy), a light `PostureTask` for the base,
  and (when the gripper is simulated) a light `PostureTask` holding its knuckle joints.
- **Damping:** there is *no* generic "damping task" in mc_rtc &mdash;
  `mc_tasks::force::DampingTask` is an admittance/force-control task that requires
  a force sensor. Damping/regularization here is the `damping` coefficient of the
  trajectory tasks (`base_task.damping`, optional; default `2*sqrt(stiffness)`)
  together with the light posture tasks.

Command contract (WbcData)
--

A single flat `std_msgs/Float64MultiArray` (32 doubles, layout in `src/WbcData.h`)
carries all fields every message:

| Field | Size | Active? | Destination |
| --- | --- | --- | --- |
| `eef_pos` | 3 | yes | arm EE `SurfaceTransformTask` target (position) |
| `eef_quat` | 4 (w,x,y,z) | yes | arm EE `SurfaceTransformTask` target orientation — **standard ROS/Hamilton** (tf2/RViz), world frame |
| `posture_arm` | 6 | yes | arm `PostureTask` target |
| `posture_base` | 3 | yes | TriOrb `PostureTask` target (joint-space base command) |
| `gripper_opening` | 1 | yes | `RobotiqGripper::setOpening` (0 = open, 1 = closed) |
| `velocity_base` | 3 (vx,vy,wyaw) | yes | base body velocity |
| `task_weights` | 4 | yes | per-task QP weight — **selects the "mode"** |
| `task_stiffness` | 4 | yes | per-task tracking gain — **compliance** |
| `task_damping_ratio` | 4 | yes | ζ when stiffness > 0 (`D = 2ζ√K`, ζ=1 critical); **absolute D** when stiffness = 0 ([gains.md](docs/gains.md)) |

The three gain vectors are ordered **`[ee, posture_arm, base, base_posture]`**.

**Modes are gains, chosen by the client.** The controller is a generic weighted-QP
executor: it never switches modes itself, it just applies whatever gains arrive (a
value `< 0` keeps the current/YAML-default value; `>= 0` is adopted, weights/stiffness
clamped `>= 0`). So the client picks behaviour by weighting tasks:

- **arm**: high `w_ee` = Cartesian (posture regularizes); high `w_posture_arm` with
  `w_ee = 0` = joint-space (`posture_arm` drives the joints).
- **base**: high `w_base` = velocity-driven (from `velocity_base`); high `w_base_posture`
  with `w_base = 0` = joint-space (`posture_base` drives `base_x/base_y/base_yaw`).
- **compliance**: lower `task_stiffness` for a soft/springy task; `task_damping_ratio`
  shapes overshoot (ζ = 1 critical; only lower it deliberately). Set a task's stiffness
  to 0 to turn it into a pure **velocity damper** (then `task_damping_ratio` is the
  absolute damping D) — e.g. immobilize the base via the base posture task; see
  [`docs/gains.md`](docs/gains.md).

Presets (`se3`, `direct`, `joint`, `compliant`) live in the Python client
(`scripts/callm_wbc_client.py`, `MODE_PRESETS` / `WbcData.set_mode()`); the active
gains are echoed back on the measured topic. A degenerate weight set that leaves the
arm (`w_ee + w_posture_arm`) or the base (`w_base + w_base_posture`) with no authority
is rejected and the previous weights kept.

Base command: **velocity is the active path.** `velocity_base` (body frame) is
integrated into the `TransformTask` target each control step, with a body-velocity
feed-forward (`refVelB`). The absolute base-pose path (`BASE_TARGET_KEY` / the
GUI *Base target [world]* marker) is kept as an inactive/overridable alternative,
selected with `base_command.mode: pose`. Exactly one path drives the base, so the
two never fight in the QP.

**Rotation convention.** Every rotation that crosses the ROS wire is **standard
ROS/Hamilton** (the same as tf2, RViz, `geometry_msgs`): `eef_quat` is the active
orientation of the Tool in world, so `(0.707,0.707,0,0)` is a real +90° about X.
Internally mc_rtc / SpaceVecAlg store the transposed *frame* rotation, so the wire↔pose
conversion transposes at exactly one boundary — the `poseFromWire()` / `wireQuat()`
helpers in `CallmWbcController.cpp` (and the same transpose the `VisualOdometryObserver`
applies to the VO pose). Route any new pose I/O through those helpers. (The `world_X_map`
observer YAML uses mc_rtc's `rpyToMat` rpy convention, as befits an mc_rtc config field.)

Commanding it
--

run() is minimal: it copies the latest ROS-buffered command into the datastore
(on the control thread), pushes the datastore values onto the live tasks / gripper,
optionally republishes measured state, and steps the QP. Targets live in the
datastore (single source of truth) and are exposed in the GUI under `CallmWbc`.

**ROS2 client.** The controller runs an `rclcpp` node on its own context and spin
thread (mirroring `explicit_compliance_controller`); the subscriber callback only
fills a mutex-guarded `WbcData` buffer, so no mc_rtc object is touched off the loop.

- command topic (default `callm_wbc/command`): a `WbcData` message drives the tasks.
- measured topic (default `callm_wbc/measured`): the controller republishes the
  current EE pose, arm/base posture, base body velocity and gripper opening.

A reference Python client is in `scripts/callm_wbc_client.py`.

An external commander can also drive the datastore keys directly:

```cpp
controller.datastore().assign<sva::PTransformd>(CallmWbcController::EE_TARGET_KEY, pose);
controller.datastore().assign<Eigen::Vector3d>(CallmWbcController::BASE_VELOCITY_KEY, {vx, vy, wyaw});
controller.datastore().assign<double>(CallmWbcController::GRIPPER_OPENING_KEY, opening);
```

Hardware stack (one loop)
--

On the real robot the controller runs under **mc_rtde**, which owns the RT loop and
drives the **UR5e arm** (position mode). The **base** and **gripper** are mc_rtc
`GlobalPlugin`s in the same loop (I/O off the RT thread). See `etc/mc_rtde_callm.yaml`.

- **Base state ← Visual Odometry (observer).** The base is velocity-controlled
  hardware, so its measured *state* is grounded in `realRobots()` by the
  **`VisualOdometryObserver`** (a state-observer, not controller code): it subscribes
  to a `geometry_msgs/PoseStamped` topic (default `/robot_pose_slam`), writes the
  TriOrb `base_x`/`base_y`/`base_yaw` joints, and reconstructs the UR5e floating base
  from the `mount` (the arm↔base contact is a QP construct and is **not** resolved for
  the real robots). An `Encoder` observer fills the arm joints. The controller then
  consumes `realRobots()` — no VO is ever written into `robots()`. How the control
  robots take up the estimate is the `feedback` mode (QP `FeedbackType`). See
  [`docs/observer.md`](docs/observer.md) and the separate `mc_visual_odometry_observer`
  package.
- **Base command → TriorbBasePlugin.** After the QP, `run()` exports the
  QP-realized base velocity (`triorb` `mbc().alpha`) rotated into the base body
  frame (on the measured/VO yaw under closed-loop feedback, else the control yaw) to
  the plugin's `Triorb::cmd_velocity` datastore key; the plugin must be configured
  `command_in_world_frame: false`.
- **Gripper → RobotiqGripperPlugin** via `RobotiqGripper::setOpening` as before.

With no VO publisher and no plugins (pure RViZ sim), run with `feedback: none`: the
base is driven by the in-QP velocity integration and nothing is exported.

The ROS command hand-off uses `try_lock`, and the VO read is in the observer (off the
control thread), so the control loop never blocks on a ROS spin thread under
mc_rtde's `SCHED_DEADLINE` scheduling.

Build
--

Built inside `mc_rtc_superbuild`. Register it with `AddProject` in
`extensions/local.cmake` and make it `DEPENDS` on the UR5e and TriOrb robot-module
projects (`mc_ur5e`, `mc_triorb_module`) and on `mc_robotiq` (the gripper plugin)
&mdash; runtime plugins, so these are build-ordering dependencies only. **On this
branch `mc_robot_tools` is NOT required** (the gripper model is not loaded), so you
can drop it from the `DEPENDS` and from `local.cmake` to avoid its build issues; the
real gripper is still driven through the `mc_robotiq` plugin. The controller links
`mc_rtc::mc_rtc_ros` for the ROS2 client (`rclcpp` + `std_msgs` come transitively).
To run, set in your global `mc_rtc.yaml`:

```yaml
MainRobot: UR5eFloatingBase
Enabled: CallmWbcController
Plugins: [RobotiqGripperPlugin]   # enables RobotiqGripper::setOpening
```

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
