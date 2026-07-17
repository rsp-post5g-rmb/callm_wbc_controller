# Migrating from contact-based attachment to `connect()`

Phase 2 write-up: the `connect()` API established from mc_rtc source, the chosen
approach, and its justification. Everything here is cited to a file/line that was
read directly; nothing is inferred from `mc_kinova`'s choice alone.

## 1. The API, verbatim

One overload only, declared on `mc_rbdyn::RobotModule` (`mc_rtc/include/mc_rbdyn/RobotModule.h:414`):

```cpp
RobotModule connect(const mc_rbdyn::RobotModule & other,
                    const std::string & this_body,
                    const std::string & other_body,
                    const std::string & prefix,
                    const ConnectionParameters & params) const;
```

Implementation: `mc_rtc/src/mc_rbdyn/RobotModule_connect.cpp:81`. Inverse
`disconnect` at `:547`. `ConnectionParameters` is a chained-setter struct at
`RobotModule.h:264-389`.

A grep over `mc_rtc/src` and `mc_rtc/include` finds **no other `connect`** related to
robots: the only other hits are `mc_control::ControllerClient::connect`
(networking, `ControllerClient.h:89`) and `mc_rtc/SignalSlot.h`. In particular there
is **no `mc_rbdyn::Robot::connect`**, no solver-level connect, and no free function.

**This is the decisive structural fact:** `connect()` consumes and produces
`RobotModule`, never `mc_rbdyn::Robot`. It cannot be applied to already-loaded robots.

## 2. Can it be called at controller time?

Two different questions hide behind "controller-level". Separate them:

### (a) Connecting robots already loaded in the controller — NOT supported

Upstream says so explicitly
(`mc_rtc/doc/_i18n/en/tutorials/advanced/new-robot.html`, "Connecting existing robot modules"):

> In any case the merged robot is a new robot module that can be loaded and used as a
> single robot in the framework. **For now, we do not provide a way to seamlessly
> replace a robot involved in the controller with a variant created by connecting it
> to another module.** This would be very useful in case of tool changing or for
> modular robotics to connect multiple actuated modules together.
>
> This feature is planned, in the meanwhile users interested in trying to do this
> themselves need to:
> - Remove all tasks and constraints in which the robot is involved from the solver
> - Remove all gui/log callbacks that involve the robot
> - Remove the robot
> - Connect the modules and load the connected variant
> - Re-add all tasks/constraints/callbacks

That five-step dance is a documented *workaround for a missing feature*, not an API.
It is what a `reset()`-time connect would require, and it is why we do not do it.

### (b) Connecting at module-assembly time, from controller code — fully supported

`MCController` has a multi-robot constructor taking the module vector
(`MCController.h:608`):

```cpp
MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robot_modules,
             double dt, ControllerParameters params = {});
```

`CallmWbcController` already feeds it from a static helper evaluated in the
member-initializer list (`CallmWbcController.cpp:108`):

```cpp
: mc_control::MCController(robotModules(rm, config), dt)
```

Function arguments are evaluated before the base-class constructor body runs, so
`robotModules()` executes **before `MCController` loads any robot**
(`MCController.cpp:135-140`). At that moment there is no robot to remove, no task to
re-add, no GUI callback to tear down — every precondition of the five-step dance is
vacuously satisfied.

So we get exactly what the doc prescribes — *"Connect the modules and load the
connected variant"* — with the connect call living entirely inside
`callm_wbc_controller`.

## 3. Decision

**Connect in `CallmWbcController::robotModules()`.** Committed.

This satisfies the task's stated preference ("If controller-level `connect()` is
supported: prefer it. It keeps this refactor inside `callm_wbc_controller`"):
no new robot-module package, no superbuild change, no edits outside the two in-scope
repos.

It differs from `mc_kinova` only in *where the code sits*. `mc_kinova` connects inside
its robot-module factory (`mc_kinova/src/module.cpp:344` `create()` →
`createVariants()` → `attachTool()` → `parent.connect(...)` at `module.cpp:112-114`),
which is the right call for *them* because they ship reusable named variants
(`"KinovaGripper"`, `"KinovaCameraGripper"`, ...) for any consumer to load. We have
exactly one rig and one consumer, so a module package would be pure overhead. The
`connect()` call itself is identical in kind.

The worked upstream example
(`mc_rtc/doc/_i18n/en/tutorials/advanced/new-robot/robot_module_connect.cpp`) does the
same thing from controller code, then `ctl.loadRobot(*pandaOnBoxRm)`. We hand the
module to the `MCController` constructor instead of `loadRobot()`, because ours
*replaces* the main robot rather than adding a fourth one.

## 4. What `connect()` does — verified from source

**It merges into one kinematic tree.** `RobotModule_connect.cpp:126-179`: `other`'s
bodies are added to `this`'s `MultiBodyGraph` (`:131-135`), then its joints
(`:137-143`) and links (`:145-156`), then a synthesized connect joint bridges
`this_body`↔`other_body` (`:157-169`), and finally `out.mb = mbg.makeMultiBody(...)`
(`:176`) produces a **single** `MultiBody`. Therefore **no coupling constraint is
needed** — rigidity is structural, not a QP objective.

Both loops start at index **1**, skipping `other`'s root joint (`:137`, `:145`) — so
`other`'s floating base is eliminated, as `RobotModule.h:404` states. Passing
`UR5eFloatingBase` as `other` is therefore safe; its free-flyer is dropped.

Other verified behaviour:

| Concern | Source | Result |
| --- | --- | --- |
| `ref_joint_order` | `:417-418` | `this`'s rjo + connect joint (**only if `dof() > 0`**) + prefixed `other`'s rjo |
| Self-collisions | `:405-414` | `other`'s `_minimalSelfCollisions` merged in, names remapped |
| Surfaces | `:476-504` | RSDF files copied; `name`/`link` attributes rewritten |
| Convex / force sensor / body sensor / gripper / device collisions | `:328`, `:366`, `:384`, `:428`, `:463` | **throw** with "provide or change the ... mapping" |
| Surface name collisions | `:476-504` | **not checked — silently overwritten** (see §6) |
| Empty prefix | `:36-43` | `prefixed_or_mapping` returns `"" + ref` = unchanged |
| Output | `:293-307`, `:521-542` | writes a real merged URDF + a reusable YAML module |

**No frame-capture-at-add-time semantics.** `connect()` is a pure module-level
operation that completes before any robot exists. This is the key difference from
`addContact`, whose ordering rule is documented at
`mc_rtc/doc/_i18n/en/tutorials/introduction/mobile-arm-controller.html:35-36`:

> We need to `addContact()` after setting up the initial position because a contact
> itself is a constraint. Moving the robot position using `posW()` ... would not move
> the frame used for `addContact()`.

That rule is **contact-specific and evaporates here**: the mounting geometry is baked
into the merged URDF and the connect joint. See §5 for the replacement ordering.

**No QP rebuild or notification.** The QP is built inside `MCController`'s constructor
from the module vector we supply; it never sees the pre-merge robots.

## 5. Consequences for this controller

Robot indices collapse from `0=ur5e, 1=triorb, 2=env/ground [,3=gripper]` to
**`0=callm (merged), 1=env/ground`**.

Because `MCController` binds its built-ins to **robot 0**:

- `kinematicsConstraint` (`MCController.cpp:211`) now covers base **and** arm joints
  ⇒ the separate `triorbKinematics_` for robot 1 is redundant → **removed**.
  Likewise `gripperKinematics_` → **removed**.
- `selfCollisionConstraint` (`:212-213`) is seeded from
  `robot_modules[0]->minimalSelfCollisions()` (`:214`) ⇒ the UR5e's arm self-collisions
  carry over automatically via `RobotModule_connect.cpp:405-414`.
- `postureTask` (`:215`) now spans base + arm + gripper joints.

The last one would collapse the `WbcData` 4-task contract
(`[ee, posture_arm, base, base_posture]`). Fix: **joint selection**, per
`mc_rtc/doc/_i18n/en/tutorials/recipes/joint-select.md` and
`PostureTask.h:46`. Three posture tasks on robot 0 with **disjoint** active sets are
mathematically identical to today's three per-robot tasks, because
`PostureTask::selectActiveJoints` is pure `dimWeight` masking
(`PostureTask.cpp:167-181`: it builds the complement and zeroes its `dimWeight`).

Nothing in `PostureTask`'s constructor is per-robot exclusive
(`PostureTask.cpp:105-121`) — it is an ordinary weighted objective. But
`name_` is derived as `"posture_" + robot.name()` (`PostureTask.cpp:36`, `:112`), so
three tasks on robot `callm` would all be named `posture_callm` and collide in the
logger/GUI. They are therefore renamed via `MetaTask::name()` (`MetaTask.h:64`), which
must happen **before** `addTask` (`MetaTask.h:61`).

Also:
- arm↔base collision moves from `addCollisions("ur5e", "triorb", ...)` to
  `addCollisions("callm", "callm", ...)`. `MCController::addCollisions` handles
  `r1 == r2` — the swap guard at `MCController.cpp:946` is skipped and it builds a
  self-collision `CollisionsConstraint` (`:961-965`).
- the runtime `ArmMount` planar surface on `mount` existed **only** to support the
  Base–Base contact → **removed**.
- the runtime gripper `Base` surface fallback existed **only** to support the
  Tool–Base contact → **removed**.

### New `reset()` ordering

The old rule ("`posW()` before `addContact()`") no longer applies — there are no
contacts left. The new rule is simply:

1. `MCController::reset(reset_data)`
2. `robot(0).posW(...)` — places the **single** merged robot's root (`odom`). The arm
   and gripper follow through the merged tree; there is nothing to keep in sync.
3. Create tasks, `reset()` them (target := current pose), `name()` them, apply joint
   selectors, `addTask`.

Step 2 no longer needs to precede step 3 for *correctness of the attachment* (there is
none to capture), but it must still precede `eeTask_->reset()` / `baseTask_->reset()`,
because those seed their target from the current pose.

## 6. Connect parameters chosen

**Arm onto base** — `triorb->connect(*rm, "mount", "base_link", "", params)`:

- `this` = triorb, so the merged root is triorb's `odom` and `base_x/base_y/base_yaw`
  keep their names (only `other`'s entities get prefixed). This is what keeps the
  observer's `planar_joints` working.
- `this_body = "mount"` — a real link (`mc_triorb_description/urdf/triorb.urdf:75`,
  at z=0.60 with Rz(-90°) via `base_to_mount`).
- `other_body = "base_link"` — the UR5e root link.
- `prefix = ""` — no collisions to avoid. triorb bodies are
  `{odom, base_x_link, base_y_link, base_yaw_link, base, mount}` and joints
  `{base_x, base_y, base_yaw, base_mount_joint, base_to_mount}`; UR5e uses
  `{base_link, shoulder_link, ..., tool0}` / `{shoulder_pan_joint, ...}`. Disjoint.
- both transforms identity — reproduces exactly today's
  `robot(0).posW(robot("triorb").bodyPosW("mount"))` plus coincident `ArmMount`/`Base`
  planar surfaces (both `PTransformd::Identity()`).
- `jointType` defaults to `Fixed` (`RobotModule.h:314`) ⇒ `dof() == 0` ⇒ the connect
  joint is **not** appended to `ref_joint_order` (`RobotModule_connect.cpp:417`).

Merged rjo = `[base_x, base_y, base_yaw] + [6 UR5e joints]` = **9 entries**.

**Gripper onto tool** — `merged.connect(*gm, "tool0", <base link>, "", params)`, following
`mc_kinova`'s `attachTool` shape (`module.cpp:112-114`), with
`X_other_connection(RotZ(M_PI))`.

`RotZ(M_PI)` is exactly `RobotiqGripperRobotModule::defaultMountingTransform()`
(`mc_robot_tools/robotiq_gripper/src/robotiq_gripper.cpp:37-40`), and the base link is
exactly `baseFrame()` (`:21-24`). We use the literals the controller already holds
rather than linking `mc_robot_tools`, because this package deliberately keeps robot
modules as **runtime** plugins, not link-time deps
(`callm_wbc_controller/src/CMakeLists.txt:6-8`). `mc_kinova` can afford the dependency;
we would be adding one for two compile-time constants.

`RotZ(M_PI)` is an involution (its own inverse), so `X_other_connection(RotZ(pi))`
reproduces today's `posW(RotZ(pi) * toolPose)` exactly.

**Surface mapping is required here.** The gripper's RSDF declares
`<planar_surface name="Base" ...>`
(`mc_robot_tools/robotiq_gripper/rsdf/robotiq_2f_85_gripper.rsdf:3`) — the same surface
name as the UR5e's (`mc_ur5e_description/rsdf/ur5e/base_link.rsdf:6`). Unlike convex
hulls and sensors, `connect()` does **not** check surface-name collisions, so this
would silently clobber. We pass
`.surfaceMapping({{"Base", "GripperBase"}})` (`RobotModule.h:372`) to rename only the
surface, leaving bodies/joints unprefixed.

## 7. mc_rtde implications (flagged, OUT OF SCOPE)

Not implemented here. Recorded so the hardware work is not surprised:

- `mc_rtde` assumes **one mc_rtc robot == one UR arm**, i.e. `robot.refJointOrder()` *is*
  the arm's 6 joints. The merged robot's rjo is 9. Specifically:
  - `mc_rtde/src/URControl.cpp:93` matches `rtdeConfig.has(robot.name())`; the merged
    robot is `callm`, not `ur5e` ⇒ no match ⇒ the `else` at `:110-112` warns and **no UR
    loop is created** (arm never driven).
  - `mc_rtde/src/URControlType.h:41-52` writes `q[i]` for `i < rjo.size()` into a
    `std::vector<double>(6)` ⇒ **heap buffer overflow** at i=6,7,8 if the name is forced
    to match.
  - `mc_rtde/src/URControl.h:91-97` reads `state_.qIn_[i]` (6 elements) for `i < rjo.size()`.
  - `mc_rtde/src/URControl.h:116-118` `memcpy`s 6 doubles and calls `setEncoderValues`
    with a 6-vector; `MCGlobalController::setEncoderValues` is a whole-vector assignment
    (`mc_rtc/src/mc_control/mc_global_controller.cpp:574-577`), so it would resize
    `encoderValues` to 6 and destroy the base slots.
  - `mc_rtde/src/URControl.cpp:130-134` has a hardcoded `qIn(6)` loop.
- The fix is a joint-subset mapping (a per-robot `joints:` list in the RTDE config,
  resolved once to mbc/rjo indices), plus read-modify-write for the encoder slice.
- **New requirement created by this refactor:** `base_x/base_y/base_yaw` are now real
  joints in the merged rjo, so something must supply their *encoder* values.
  `TriorbBasePlugin` currently writes body-sensor position/orientation
  (`mc_triorb/src/TriorbBasePlugin.cpp:224-225`), not encoders.

Simulation is unaffected: the ticker seeds from the module's `_stance`.
