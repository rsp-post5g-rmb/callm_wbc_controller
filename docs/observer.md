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
**The observer therefore writes the three planar joints of `realRobot("triorb")`
(+ `forwardKinematics`/`forwardVelocity`), not `posW()`.**

## The subtlety that shaped this design: contacts are QP-only

The arm is bolted to the base by a rigid contact `triorb::ArmMount ↔ ur5e::Base`.
It is natural to expect that contact to place the UR5e floating base once the base
moves. It does — **but only for the control copy.** The contact is a QP
constraint, enforced during the solve on `robots()`
(`mc_rtc/src/mc_solver/TasksQPSolver.cpp` — constraints updated at `:312`, solved
at `:320`). **`realRobots()` never pass through the QP**; no solver, no contact
resolution runs on them. `runClosedLoop` does not *solve* the real copy, it
*copies* `realRobot.mbc().q` into the control copy (`TasksQPSolver.cpp:304`).

Consequence: nothing positions `realRobot("ur5e")`'s floating base. The
`EncoderObserver` only writes actuated joints and runs FK from the *existing*
floating base (`mc_rtc/src/mc_observers/EncoderObserver.cpp:128-143`) — the
tutorial states this plainly ("still missing the floating base"). So when VO moves
`realRobot("triorb")`, the arm's real copy would detach from the base's real copy.

This is a consequence of the rig being **two robots joined by a contact** rather
than one kinematic tree. In the real copy the contact must be reproduced
explicitly.

## Full realRobots consistency (the fix)

Because the UR5e base is *rigidly* bolted to the TriOrb `mount`, its floating-base
pose is a deterministic function of the base pose — a reconstruction, not a
measurement. The observer reconstructs it (mirrors `reset()`'s
`posW(bodyPosW("mount"))`, `CallmWbcController.cpp:542`):

```cpp
realRobot("ur5e").posW( realRobot("triorb").bodyPosW("mount") );
realRobot("ur5e").forwardKinematics();
```

Result — `realRobots()` is fully consistent:

| realRobots component | source |
|---|---|
| `triorb` base joints `base_x/base_y/base_yaw` | **VisualOdometry observer** (VO) |
| `ur5e` arm joints | **Encoder observer** |
| `ur5e` floating base | **VO observer, reconstructed from `triorb` `mount`** |

With the real copy coherent for *both* robots, the QP feedback mode (`FeedbackType`)
becomes a free, safe choice (see below). This reconstruction is **Option 1**:
folded into the VisualOdometry observer (config `attach:` block) so the arm base is
re-derived from the base pose the same observer just set.

## Observer pipeline

```yaml
ObserverPipelines:
  - name: CallmObservers
    gui: true
    log: true
    observers:
      - type: VisualOdometry     # updates triorb base (VO) + reconstructs ur5e floating base
        update: true
        gui: true
        robot: triorb
        topic: /robot_pose_slam
        expected_frame_id: ""            # empty = skip header.frame_id check
        world_X_map: {translation: [0,0,0], rotation: [0,0,0]}   # world <- map (VO) transform
        height: 0.0                      # fixed z when planar
        planar: true                     # take x,y,yaw from VO; z,roll,pitch = fixed
        timeout: 0.5                     # s; older than this -> run() fails -> last pose held
        max_jump: 0.5                    # m; larger step accepted (loop closure) but warned + flagged
        attach:                          # Option 1: reconstruct the arm floating base
          robot: ur5e
          mount_body: mount              # body on `robot: triorb` the arm base is bolted to
      - type: Encoder            # updates ur5e arm joints from encoders
        update: true
```

Ordering `VisualOdometry -> Encoder` lets the arm-joint FK use the freshly set
floating base. (The observer also re-runs `ur5e` FK after setting the FF, so the
order is robust either way.) The pipeline summary line at startup prints the
observers (bracketed entries do not update the real robot).

## Per-tick data flow

1. **Pipeline `run()`+`update()`** (before `MCController::run()`):
   - VO observer: `realRobot("triorb")` base joints ← latest cached VO pose (+FK/FV);
     then `realRobot("ur5e").posW(realRobot("triorb").bodyPosW("mount"))` (+FK).
   - Encoder observer: `realRobot("ur5e")` arm joints (+FK).
   - If VO is stale, VO `run()` returns `false`; the pipeline **skips `update()`**
     (`ObserverPipeline.cpp:170`) so the last good real pose is held.
2. **Controller `run()`**:
   - ROS command → datastore → live tasks/gains.
   - Base-target re-base anchor read from `realRobot("triorb")` when VO is alive
     (else the control base, preserving pure-sim motion).
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
`realRobot("triorb")`, which in **pure sim with no VO publisher** holds the reset
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
base pose" — must mean *where the base actually is*. Its source depends on VO:

| Situation | Anchor source | Why |
|---|---|---|
| VO alive (hardware) | `realRobot("triorb").bodyPosW("base")` | the observer wrote the true pose this tick; the control base is not the measurement |
| VO absent (pure sim) | `robots().robot("triorb").bodyPosW("base")` | `realRobot` is frozen at the reset pose; only the control base moves (QP integration) |

```cpp
anchor = VO_alive ? realRobot("triorb").bodyPosW("base")
                  : robots().robot("triorb").bodyPosW("base");
```

Anchoring to `realRobot` in sim would peg the target to *(frozen reset pose + one
tick)* every tick — a fixed point — and the base would never progress. The guard
prevents that.

**Timing — why `FeedbackType` does not cover this.** Within one tick:

1. observer pipeline updates `realRobot("triorb")` ← fresh VO
2. controller `run()` **pre-QP**: `integrateBaseVelocity()` reads the anchor **here**
3. `MCController::run(fType)` → the QP, where a closed-loop `fType` resets the
   control base from `realRobot` (`TasksQPSolver.cpp:304`)

The anchor is read at step 2, but the closed-loop reset only happens at step 3 — so
at step 2 the control base still holds *last* tick's value. Reading `realRobot`
directly (refreshed at step 1) is the only way to anchor off the current
measurement, independent of the chosen feedback mode.

**"VO alive"** is the observer's health flag: a fresh message arrived within
`timeout`. It is exposed on the datastore (`VO::isAlive`, mirroring
`SLAMObserver.cpp:239`) and drives both the staleness failure mode and this anchor
switch, so a VO dropout automatically falls back to the control-base anchor.

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
- Base-target re-base anchor is read from `realRobot("triorb")` when VO is alive.
- `exportBaseVelocity()` still exports the control (commanded) base velocity.

The observer itself lives in a separate package (`VisualOdometryObserver`,
`EXPORT_OBSERVER_MODULE("VisualOdometry", ...)`) installed to the mc_rtc observers
path; see that package's README for build/config-key details.
