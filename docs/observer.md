# State Observation: Visual Odometry for the TriOrb base

How the CallmWbcController grounds the mobile base from visual odometry (VO,
ORB-SLAM3 on `/robot_pose_slam`) the mc_rtc way: through an **observer** that
writes to `realRobots()`, not by writing the estimate into the control robot.

## Two robot copies

mc_rtc keeps two copies of every loaded robot:

| Copy | Accessor | Written by | Meaning |
|---|---|---|---|
| control | `robots()` / `robot(name)` | the QP | *desired* state; the QP integrates this forward |
| real | `realRobots()` / `realRobot(name)` | **observers** | *measured* state estimate |

VO is a measurement, so it belongs in the **real** copy. The old design wrote it
straight into the control copy (`applyBaseState()` injected the SLAM pose into
`robots().robot("triorb")`); that is what this observer replaces.

## Why the base needs an observer at all

The TriOrb is velocity-controlled hardware: we command body velocities and the
real base ends up somewhere the control copy cannot predict (slip, latency). VO
measures the true planar pose `(x, y, yaw)`. That measurement has to reach the
real copy every tick so the controller can plan against the true base pose (the
arm's end-effector target is in the world frame).

The TriOrb is modelled **fixed-base with explicit planar joints**
(`base_x` prismatic, `base_y` prismatic, `base_yaw` continuous;
`mc_triorb_description/urdf/triorb.urdf`). Its `mb().joint(0)` is `Fixed`, so
`posW()` cannot encode the planar pose — the pose lives in those three joints.
**The observer therefore writes the three planar joints of `realRobot()`
(+ `forwardKinematics`/`forwardVelocity`), not `posW()`.**

## Historical note: why this used to be harder

This design once had to work around a real subtlety, and it is worth recording why it
no longer does.

When the arm was a **separate robot** bolted on by a rigid contact
(`triorb::ArmMount ↔ ur5e::Base`), that contact placed the UR5e floating base — **but
only for the control copy.** A contact is a QP constraint, enforced during the solve on
`robots()` (`mc_rtc/src/mc_solver/TasksQPSolver.cpp` — constraints updated at `:312`,
solved at `:320`). **`realRobots()` never pass through the QP**; no solver, no contact
resolution runs on them. `runClosedLoop` does not *solve* the real copy, it *copies*
`realRobot.mbc().q` into the control copy (`TasksQPSolver.cpp:304`).

So nothing positioned `realRobot("ur5e")`'s floating base — the `EncoderObserver` only
writes actuated joints and runs FK from the *existing* floating base
(`mc_rtc/src/mc_observers/EncoderObserver.cpp:128-143`). When VO moved
`realRobot("triorb")`, the arm's real copy detached from the base's. The observer had to
reproduce the contact explicitly, via an `attach:` block that did
`realRobot("ur5e").posW(realRobot("triorb").bodyPosW("mount"))`.

**That whole problem was an artifact of the rig being two robots joined by a contact
rather than one kinematic tree.**

## Full realRobots consistency (now free)

Since the migration to `mc_rbdyn::RobotModule::connect` (see
[connect_migration.md](connect_migration.md)) there is exactly **one** robot: the arm
hangs off the base through a fixed connect joint in the *same* `MultiBody`. Writing the
base planar joints and calling `forwardKinematics()` therefore carries the arm — and the
gripper — automatically. There is no floating base left to reconstruct, and the `attach:`
option is obsolete (it warns if still configured).

`realRobots()` is fully consistent with no special handling:

| realRobots component | source |
|---|---|
| base joints `base_x/base_y/base_yaw` of `callm` | **VisualOdometry observer** (VO) |
| arm joints of `callm` | **Encoder observer** |
| arm/gripper link poses | **merged-tree FK** — implied by the two above |

With the real copy coherent, the QP feedback mode (`FeedbackType`) is a free, safe
choice (see below).

## Observer pipeline

```yaml
ObserverPipelines:
  - name: CallmObservers
    gui: true
    log: true
    observers:
      - type: VisualOdometry     # updates the merged robot's base planar joints from VO
        update: true
        gui: true
        robot: callm                     # the merged robot (connect.name)
        topic: /robot_pose_slam
        expected_frame_id: ""            # empty = skip header.frame_id check
        world_X_map: {translation: [0,0,0], rotation: [0,0,0]}   # world <- map (VO) transform
        height: 0.0                      # fixed z when planar
        planar: true                     # take x,y,yaw from VO; z,roll,pitch = fixed
        timeout: 0.5                     # s; older than this -> run() fails -> last pose held
        max_jump: 0.5                    # m; larger step accepted (loop closure) but warned + flagged
        planar_joints: [base_x, base_y, base_yaw]
      - type: Encoder            # updates the arm joints from encoders
        update: true
```

Ordering `VisualOdometry -> Encoder` lets the arm-joint FK run on top of the freshly
set base pose. Both observers now write the *same* robot and each re-runs FK, so the
order is robust either way. The pipeline summary line at startup prints the observers
(bracketed entries do not update the real robot).

## Per-tick data flow

1. **Pipeline `run()`+`update()`** (before `MCController::run()`):
   - VO observer: `realRobot("callm")` base joints ← latest cached VO pose (+FK/FV).
     The FK carries the arm and gripper too — they are in the same tree.
   - Encoder observer: `realRobot("callm")` arm joints (+FK).
   - If VO is stale, VO `run()` returns `false`; the pipeline **skips `update()`**
     (`ObserverPipeline.cpp:170`) so the last good real pose is held.
2. **Controller `run()`**:
   - ROS command → datastore → live tasks/gains.
   - Base-target re-base anchor read from `realRobot()` only under closed-loop
     feedback with fresh VO (`useMeasuredBase()`), else the control base (so open-loop
     VO does not move the control base; pure sim keeps moving).
   - `MCController::run(fType)` — the QP; under a closed-loop `fType` it resets the
     control copy from the (now consistent) real copy before solving.
   - `exportBaseVelocity()` — the QP-realised **control** base velocity (the
     command) → `Triorb::cmd_velocity`.

## Feedback mode (how the control robot consumes realRobots)

Selected by which `MCController::run(FeedbackType)` overload the controller calls
(`mc_solver/QPSolver.h:53`; `MCController::run()` defaults to `None`,
`MCController.cpp:815`). The precise per-mode QP semantics (which state the QP is built
at vs integrated from) are in [`qp_feedback.md`](qp_feedback.md). Exposed as a controller
config key `feedback`:

| `feedback` | `FeedbackType` | Behaviour | Use |
|---|---|---|---|
| `none` | `None` / `OpenLoop` | control copy integrates commands; realRobots ignored by the QP | pure sim (no VO) |
| `joints` | `Joints` | actuated joints reset from encoders; base + floating base untouched | arm closed-loop, base open-loop |
| `observed` | `ObservedRobots` / `ClosedLoop` | resets **all** robots from realRobots, integrates from control state | hardware, full closed loop |
| `observed_real` | `ClosedLoopIntegrateReal` | as above but integrates from the real state | rarely; forces base velocity to the (zero) observed value in v1 |

Because realRobots is now consistent, `observed` no longer feeds the arm a stale
floating base. Caveat: `observed`/`observed_real` reset the base from
`realRobot()`, which in **pure sim with no VO publisher** holds the reset
pose → the base would freeze. Use `none` in that case. This is a mode choice, not
a bug.

## Base-target re-base anchor (the VO-alive guard)

The base is not driven by a fixed target: `integrateBaseVelocity()` re-computes the
base `TransformTask` target every tick as

```
target = (current base pose) ⊕ (commanded velocity × dt)
```

so the target is re-anchored to where the base is *now* and stepped forward one
tick (this is the anti-wind-up re-basing, see
[`base_velocity_target.md`](base_velocity_target.md)). The **anchor** — "current
base pose" — must be *the state the QP builds at this tick*, which is decided by the
**feedback mode**, not merely by whether VO is publishing:

| Situation (`useMeasuredBase()`) | Anchor source | Why |
|---|---|---|
| closed-loop (`observed`/`observed_real`) **and** VO fresh | `realRobot().bodyPosW("base")` | the QP resets the control base to `realRobot` before solving, so the target must be relative to `realRobot` (`TasksQPSolver.cpp:304`) |
| open-loop (`none`/`joints`), or VO stale | `robot().bodyPosW("base")` | the control base is **not** grounded to `realRobots`, so it re-bases off itself; VO updates `realRobots()` only and does **not** move the control base |

```cpp
useMeasuredBase() = isClosedLoop(feedback) && voAlive();
anchor = useMeasuredBase() ? realRobot().bodyPosW("base")
                           : robot().bodyPosW("base");
```

Gating on the feedback mode (not just VO) is deliberate: under `feedback: none` with VO
publishing, anchoring to `realRobot` would make the base task servo the **control** base
toward the VO pose — surprising, since open-loop should leave the control base
independent of VO. In pure sim it would also peg the target to *(frozen reset pose + one
tick)* and stall the base. The guard prevents both.

**Timing — why `FeedbackType` does not cover this.** Within one tick:

1. observer pipeline updates `realRobot()` ← fresh VO
2. controller `run()` **pre-QP**: `integrateBaseVelocity()` reads the anchor **here**
3. `MCController::run(fType)` → the QP, where a closed-loop `fType` resets the
   control base from `realRobot` (`TasksQPSolver.cpp:304`)

The closed-loop reset only happens at step 3, but the anchor is read at step 2 — where
the control base still holds *last* tick's value. So **when closed-loop is selected**,
reading `realRobot` at step 2 (refreshed at step 1) is the only way to anchor off the
pose the QP is about to build at. When open-loop is selected, the QP builds at the
control base itself, so that is the correct anchor — hence `useMeasuredBase()` gates on
the feedback mode, not just on VO.

**"VO alive"** is the observer's health flag: a fresh message arrived within
`timeout`, exposed on the datastore (`VO::isAlive`, mirroring `SLAMObserver.cpp:239`).
It is one of the two `useMeasuredBase()` conditions (with a closed-loop feedback mode)
and also drives the staleness failure mode, so a VO dropout falls back to the
control-base anchor.

## Topic contract

- Message: `geometry_msgs/msg/PoseStamped` on `topic` (default `/robot_pose_slam`).
- Frame: the pose is the **TriOrb base link** pose in the VO/map frame.
  `world_X_map` (an `sva::PTransformd` from config) maps map → controller world and
  is applied before writing. Mind the `sva::PTransformd` transpose convention.
- Fields:
  | field | use |
  |---|---|
  | `pose.position.{x,y}` | base translation x, y |
  | `pose.orientation` | yaw only when `planar: true` (via `mc_rbdyn::rpyFromMat`, not a hand-rolled atan2) |
  | `pose.position.z`, roll, pitch | discarded when `planar`; z ← `height` |
  | `header.stamp` | staleness check against the ROS clock (zero/stuck stamps ⇒ stale) |
  | `header.frame_id` | logged once; warn-not-fail if it differs from `expected_frame_id` |
- Quaternion → pose uses the sva/Eigen path from `SLAMObserver.cpp`
  (`sva::conversions::fromHomogeneous`, `mc_rbdyn::rpyFromMat`/`rpyToMat`).

## Threading

The subscription runs on the observer's **own** `rclcpp::Context` + node +
`SingleThreadedExecutor` + spin thread (the pattern already proven in the
controller's `setupRos`, with `auto_initialize_logging(false)` to defer logging to
mc_rtc's ROS plugin). The callback only fills a mutex-guarded pose buffer; `run()`
does a `try_lock` read and never spins. The control loop never blocks on ROS.

## Failure modes

| Condition | Behaviour |
|---|---|
| No message within `timeout` | `run()` returns `false` (+error string); `update()` skipped; **last good pose held** (no extrapolation) |
| Zero / monotonically stuck `header.stamp` | treated as stale (as above) |
| Pose jump > `max_jump` | **accepted** (loop closures are real corrections) but warned once and flagged on the datastore (`VO::jump`) — never silently absorbed |
| `header.frame_id` ≠ `expected_frame_id` | warn once, keep running |
| No VO publisher (pure sim) | real base holds reset pose; run the QP with `feedback: none` so the base moves by velocity integration |

`PoseStamped` carries no covariance, so there is **no confidence signal**;
staleness and jump detection are the only health signals available.

## Velocity (v1)

The observer sets the base floating-base velocity to **zero** and logs a TODO —
ORB-SLAM3 loop closures cause pose jumps and finite-differencing them would inject
velocity spikes. Downstream impact: with `feedback: observed` the control base
`alpha` is restored from the control state on integration (`TasksQPSolver.cpp:328`),
so the commanded velocity trajectory is preserved; only `feedback: observed_real`
would actually force the zero velocity into the base. A filtered velocity (e.g.
Savitzky–Golay, as `SLAMObserver` does for pose) is the v2 alternative.

## What changed in the controller

- Removed: `applyBaseState()` (the VO→control-robot write) and its `run()` call.
- Removed: the controller's own SLAM subscription / `handleSlamPose` / `SlamPose` /
  `base_state.from_slam` machinery — the observer now owns the VO subscription.
- `MCController::run()` call takes a configurable `FeedbackType` (`feedback` key).
- Base-target re-base anchor and export yaw read `realRobot()` only under
  closed-loop feedback with fresh VO (`useMeasuredBase()`); open-loop leaves the control
  base independent of VO.
- `exportBaseVelocity()` still exports the control (commanded) base velocity.

The observer itself lives in a separate package (`VisualOdometryObserver`,
`EXPORT_OBSERVER_MODULE("VisualOdometry", ...)`) installed to the mc_rtc observers
path; see that package's README for build/config-key details.
