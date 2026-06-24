CallmWbcController
==

Whole-body controller ([mc_rtc]) that coordinates a **UR5e arm** mounted on a
**TriOrb omnidirectional base** in a single QP. It is adapted from the mc_rtc
*Mobile arm controller* tutorial, replacing Dingo with the TriOrb module and
restructuring it as a reusable WBC controller (no scripted phase machine).

Model
--

| Index | Robot | Role |
| --- | --- | --- |
| 0 | `UR5eFloatingBase` | main robot, arm with a floating base |
| 1 | `triorb` | the mobile base |
| 2 | `env/ground` | visual reference only |

- The UR5e floating base is rigidly attached to the TriOrb `mount` link through a
  Base&harr;Base contact (all 6 dof constrained). When the QP moves the base the arm
  follows; when the end-effector task pulls the hand, the solver distributes motion
  across both the arm joints and the base dof &mdash; that is the whole-body
  coordination, with no explicit IK.
- **No base&harr;ground dof-masked contact is used.** The TriOrb module is *fixed-base*
  with explicit planar joints (`base_x`, `base_y`, `base_yaw`), so planar motion is
  intrinsic to the kinematics and solved directly by the QP through those joints.
  (This differs from the Dingo-based tutorial, where the floating base needed a
  planar contact to the ground.)
- The TriOrb description ships no RSDF surfaces, so the attachment surface
  `ArmMount` is declared on the `mount` link at runtime, in the constructor.

Tasks / constraints
--

- `contactConstraint`, `kinematicsConstraint` (UR5e), `selfCollisionConstraint` (UR5e),
  plus a per-robot `KinematicsConstraint` for the TriOrb.
- `SurfaceTransformTask` on the UR5e `Tool` surface (arm end-effector).
- `TransformTask` on the TriOrb `base` body (base command).
- `PostureTask` for the UR5e (arm redundancy) and a light `PostureTask` for the base.

Commanding it
--

`run()` only copies the commanded targets into the tasks and steps the QP. Targets
live in the datastore (single source of truth) and are exposed in the GUI under the
`CallmWbc` category:

- `EE target [world]` &mdash; interactive `sva::PTransformd` marker.
- `Base target [world]` / `Base target [x, y, yaw]` &mdash; base pose (keep it planar:
  z = 0.30, level).

An external commander (teleop / diffusion policy) can drive them live without the GUI:

```cpp
controller.datastore().assign<sva::PTransformd>(CallmWbcController::EE_TARGET_KEY, pose);
controller.datastore().assign<sva::PTransformd>(CallmWbcController::BASE_TARGET_KEY, basePose);
```

Build
--

Built inside `mc_rtc_superbuild`. Register it with `AddProject` in
`extensions/local.cmake` and make it `DEPENDS` on the UR5e and TriOrb robot-module
projects (they are runtime plugins, hence build-ordering dependencies only). The
controller's own `CMakeLists.txt` uses the mc_rtc `add_controller` macro. To run, set
in your `mc_rtc.yaml`:

```yaml
MainRobot: UR5eFloatingBase
Enabled: CallmWbcController
```

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
