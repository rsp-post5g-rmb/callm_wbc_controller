# `mc_solver::FeedbackType` — QP semantics in math notation

Grounded in:

| File | Relevant lines |
|---|---|
| `include/mc_solver/QPSolver.h` | `53–73` (enum), `246` (`run`), `331` (`run_impl`) |
| `src/mc_solver/QPSolver.cpp` | `42–43` (realRobots construction), `122–125` (`run` → `run_impl`) |
| `src/mc_solver/TasksQPSolver.cpp` | `146–182` (dispatch), `184–209` (open loop), `211–281` (joints), `283–345` (closed loop) |
| `src/mc_solver/TVMQPSolver.cpp` | `94–112` (dispatch), `114–140`, `142–196`, `198–243`, `245–252` (`updateRobot`) |

Both backends dispatch identically. `FeedbackType` selects **two states**, nothing else:

1. $q^{\text{qp}}, \alpha^{\text{qp}}$ — the state the QP is **built at** (Jacobians, task errors, constraint linearization).
2. $q^{\text{int}}, \alpha^{\text{int}}$ — the state the solution is **integrated from**.

The QP formulation, the tasks, and the constraints are untouched by the enum.

---

## 1. Notation

| Symbol | Code |
|---|---|
| $k$ | control tick |
| $\delta t$ | `QPSolver::timeStep` |
| $q_k, \alpha_k$ | **control** robot state — `robots().robot(i).mbc().q` / `.alpha` |
| $\hat q_k, \hat\alpha_k$ | **real** robot state — `realRobots().robot(i).mbc().q` / `.alpha`, written only by the observer pipeline |
| $q^{\text{enc}}_k$ | `robot.encoderValues()`, indexed by `refJointOrder()` |
| $\ddot q$ | QP decision variable — `alphaD` |
| $\lambda$ | contact-force decision variable |
| $e_i(q) = x^d_i - x_i(q)$ | task-$i$ feature error |
| $\oplus$ | `rbd::eulerIntegration` configuration update (Lie group on the free-flyer) |

Index $i$ runs over **all robots** in `robots_p`. `realRobots_p` is index-aligned with
`robots_p` by construction (`QPSolver.cpp:42–43` copies every robot at solver
construction), which is what makes `realRobots().robot(i)` valid in the closed-loop
path.

---

## 2. The QP (identical in every mode)

$$
(\ddot q^\star_k, \lambda^\star_k) = \arg\min_{\ddot q, \lambda}
\sum_i w_i \left\| J_i(q^{\text{qp}}_k)\ddot q + \dot J_i(q^{\text{qp}}_k, \alpha^{\text{qp}}_k)\alpha^{\text{qp}}_k - \ddot x^{\text{ref}}_i \right\|^2
$$

$$
\ddot x^{\text{ref}}_i = \ddot x^d_i + K_{d,i}\big(\dot x^d_i - J_i(q^{\text{qp}}_k)\alpha^{\text{qp}}_k\big) + K_{p,i}\, \underbrace{e_i(q^{\text{qp}}_k)}_{\substack{\text{the error the QP}\\\text{believes it has}}}
$$

subject to, with a `DynamicsConstraint` present,

$$
M(q^{\text{qp}}_k)\ddot q + N(q^{\text{qp}}_k, \alpha^{\text{qp}}_k) = S^\top\tau + \sum_c J_c(q^{\text{qp}}_k)^\top\lambda_c
$$

plus bounds, contacts, collisions — all evaluated at $q^{\text{qp}}_k$.

The build happens through the two loops present in every path:

```cpp
for(auto & c : constraints_) { c->update(*this); }   // reads robots_p
for(auto & t : metaTasks_)   { t->update(*this); t->incrementIterInSolver(); }
```

`update(*this)` reads `robots()`, i.e. the **control** `Robots` instance. This is the
crux: the closed-loop modes do not point the tasks at `realRobots()` — they
**overwrite `robots()` with `realRobots()` content** before calling `update`, and put
the old content back afterwards. Task/constraint code never knows.

## 3. The integrator (identical in every mode)

```cpp
solver_.updateMbc(robot.mbc(), i);   // Tasks:  writes alphaD* into mbc
robot.eulerIntegration(timeStep);    // both backends
robot.forwardKinematics(); robot.forwardVelocity(); robot.forwardAcceleration();
```

$$
\alpha_{k+1} = \alpha^{\text{int}}_k + \ddot q^\star_k\,\delta t
\qquad
q_{k+1} = q^{\text{int}}_k \oplus \Delta(\alpha^{\text{int}}_k, \ddot q^\star_k, \delta t)
$$

The exact quadrature $\Delta$ lives in `RBDyn/src/RBDyn/EulerIntegration.cpp` and is
irrelevant to the feedback argument. What matters: **$q_{k+1}$ is anchored at
$q^{\text{int}}_k$.** Both backends guard integration with `nrDof() > 0`, so zero-DoF
robots (ground/env) are skipped.

This writes `robots()`. **The solver never writes `realRobots()`.**

---

## 4. The five implemented modes

### `None` / `OpenLoop` → `runOpenLoop()`

$$
q^{\text{qp}}_k = q_k,\;\; \alpha^{\text{qp}}_k = \alpha_k
\qquad
q^{\text{int}}_k = q_k,\;\; \alpha^{\text{int}}_k = \alpha_k
$$

No save, no overwrite, no restore. Sensors appear nowhere. The recursion is a closed
discrete double integrator:

$$
\begin{bmatrix}q_{k+1}\\\alpha_{k+1}\end{bmatrix} =
\begin{bmatrix}I & \delta t\,I\\ 0 & I\end{bmatrix}
\begin{bmatrix}q_k\\\alpha_k\end{bmatrix} + \begin{bmatrix}*\\ \delta t\,I\end{bmatrix}\ddot q^\star_k
$$

The error driven to zero is $e_i(q_k)$ — the error of a *simulated* robot. If the plant
does not track $q_k$, the QP never finds out.

### `Joints` → `runJointsFeedback(false)`

$$
q^{\text{qp}}_k = \big(\underbrace{q^{\text{fb}}_k}_{\text{commanded}},\; \underbrace{q^{\text{enc}}_k}_{\text{measured}}\big),
\;\; \alpha^{\text{qp}}_k = \alpha_k
\qquad
q^{\text{int}}_k = q_k,\;\; \alpha^{\text{int}}_k = \alpha_k
$$

```cpp
control_q_[i] = robot.mbc().q;  control_alpha_[i] = robot.mbc().alpha;   // save
...
robot.mbc().q[jI][0] = encoders[j];   // per refJointOrder(), 1-DoF joints only
...                                    // build + solve
robot.mbc().q = control_q_[i];  robot.mbc().alpha = control_alpha_[i];   // restore
```

Two structural facts follow directly from that write:

- **The floating base is never touched.** The loop iterates `refJointOrder()`, which
  excludes the free-flyer. $q^{\text{fb}}$ in $q^{\text{qp}}$ stays commanded.
- **Only 1-DoF joints.** The `[0]` index is the source of the in-tree
  `// FIXME Not correct for every joint types`.

If `encoders.size() == 0` the overwrite block is skipped entirely and the mode
silently degrades to open loop for that robot — the save/restore still runs, so it is
a no-op round trip.

### `JointsWVelocity` → `runJointsFeedback(true)`

Adds, to the above,

$$
\alpha^{\text{qp}}_k = \Big(\alpha^{\text{fb}}_k,\; \tfrac{q^{\text{enc}}_k - q^{\text{enc}}_{k-1}}{\delta t}\Big)
$$

Raw finite difference, no filter (`encoders_alpha_[i][j] = (encoders[j] -
prev_encoders_[i][j]) / timeStep`). $q^{\text{int}}, \alpha^{\text{int}}$ unchanged.
Same floating-base blind spot. The Tasks backend logs this as `alphaIn` for robot 0.

### `ObservedRobots` / `ClosedLoop` → `runClosedLoop(true)`

$$
\boxed{q^{\text{qp}}_k = \hat q_k,\;\; \alpha^{\text{qp}}_k = \hat\alpha_k}
\qquad
\boxed{q^{\text{int}}_k = q_k,\;\; \alpha^{\text{int}}_k = \alpha_k}
$$

```cpp
if(integrateControlState) { control_q_[i] = robot.mbc().q;  control_alpha_[i] = robot.mbc().alpha; }
robot.mbc().q     = realRobot.mbc().q;        // FULL vector — free-flyer included
robot.mbc().alpha = realRobot.mbc().alpha;
robot.forwardKinematics(); robot.forwardVelocity(); robot.forwardAcceleration();
// build + solve
if(integrateControlState) { robot.mbc().q = control_q_[i];  robot.mbc().alpha = control_alpha_[i]; }
// updateMbc + eulerIntegration
```

Whole-vector assignment, so unlike `Joints` this **does** include the floating base.

$$
\alpha_{k+1} = \alpha_k + \ddot q^\star_k(\hat q_k, \hat\alpha_k)\,\delta t
\qquad
q_{k+1} = q_k \oplus \Delta(\alpha_k, \ddot q^\star_k, \delta t)
$$

Header wording: *"closed loop w.r.t realRobots … and integrate over the **control**
state"* (`QPSolver.h:63–65`).

### `ClosedLoopIntegrateReal` → `runClosedLoop(false)`

$$
\boxed{q^{\text{qp}}_k = \hat q_k,\;\; \alpha^{\text{qp}}_k = \hat\alpha_k}
\qquad
\boxed{q^{\text{int}}_k = \hat q_k,\;\; \alpha^{\text{int}}_k = \hat\alpha_k}
$$

Same function, `integrateControlState == false`: nothing is saved and nothing is
restored, so at integration time `robot.mbc().q` still holds `realRobot.mbc().q`.

$$
\alpha_{k+1} = \hat\alpha_k + \ddot q^\star_k\,\delta t
\qquad
q_{k+1} = \hat q_k \oplus \Delta(\hat\alpha_k, \ddot q^\star_k, \delta t)
$$

Header wording: *"… integrate over the **real** state"* (`QPSolver.h:68–70`). Added in
mc_rtc PR #240.

### `SkipQP` — **not handled in either backend**

`SkipQP` exists at `QPSolver.h:72` but appears in neither `TasksQPSolver::run_impl`
nor `TVMQPSolver::run_impl`. Both switches fall through to:

```cpp
default:
  mc_rtc::log::error("FeedbackType set to unknown value");
  success = false;   // TVM: return false
```

So reaching `QPSolver::run(SkipQP)` is an error path, not a feature. It must be
intercepted **upstream**, before `solver().run(fType)` is called — check
`MCController::run(FeedbackType)` in `src/mc_control/MCController.cpp`. Same for
`Backend::Unset`.

---

## 5. Summary

| Mode | → | $q^{\text{qp}}$ | $\alpha^{\text{qp}}$ | $q^{\text{int}}$ | $\alpha^{\text{int}}$ | Floating base observed? |
|---|---|---|---|---|---|---|
| `None`/`OpenLoop` | `runOpenLoop()` | $q_k$ | $\alpha_k$ | $q_k$ | $\alpha_k$ | no |
| `Joints` | `runJointsFeedback(false)` | $q^{\text{fb}}_k \oplus q^{\text{enc}}_k$ | $\alpha_k$ | $q_k$ | $\alpha_k$ | no |
| `JointsWVelocity` | `runJointsFeedback(true)` | $q^{\text{fb}}_k \oplus q^{\text{enc}}_k$ | FD of $q^{\text{enc}}$ | $q_k$ | $\alpha_k$ | no |
| `ObservedRobots`/`ClosedLoop` | `runClosedLoop(true)` | $\hat q_k$ | $\hat\alpha_k$ | $q_k$ | $\alpha_k$ | **yes** |
| `ClosedLoopIntegrateReal` | `runClosedLoop(false)` | $\hat q_k$ | $\hat\alpha_k$ | $\hat q_k$ | $\hat\alpha_k$ | **yes** |
| `SkipQP` | — | error path | | | | |

The `bool` argument is exactly the $q^{\text{int}}$ selector: `true` → integrate the
command, `false` → integrate the estimate.

---

## 6. Why `ObservedRobots` corrects task error but not command drift

Define the command–estimate mismatch $\epsilon_k \triangleq q_k \ominus \hat q_k$.

Under `runClosedLoop(true)`:

- **Task error is corrected.** $e_i(\hat q_k)$ is the true error, so $K_{p,i}e_i(\hat
  q_k)$ genuinely pulls the plant toward $x^d_i$. This is real feedback.
- **The command state is not.** $q_{k+1} = q_k \oplus \cdots$. The QP computes
  $\ddot q^\star$ correct *for a robot at $\hat q_k$* and applies it to a state at
  $q_k$. Nothing in the cost or constraints penalises $\epsilon$; the restore step
  discards $\hat q_k$ before integration. There is no dynamics on $\epsilon$ at all —
  not a slow one, none. It is a free integrator seeded once at `reset()`.

Under `runClosedLoop(false)`, $q_{k+1}$ is built from $\hat q_k$, so $\epsilon$ is
re-zeroed every tick and drift is structurally impossible. The price is that
*everything* in $\hat q_k$ — observer noise, latency, and any **discontinuity** —
lands directly in the command:

$$
\|q_{k+1} - q_k\| \approx \|\hat q_k - q_k\| + O(\delta t)
$$

A step in $\hat q_k$ (loop closure, MOCAP relabel, observer reset) becomes a step in
the command within one tick, i.e. an unbounded implied velocity. Under
`ObservedRobots` the same step only perturbs $e_i$, and the response is bounded by
$K_{p,i}$ and the QP's own velocity/acceleration limits.

That trade — bounded response to a jumpy estimator vs. zero steady-state command drift
— is the entire design decision behind the two modes. It is also the substance of
BaselineWalkingController issue #39.

---

## 7. Details worth remembering

- **Feedback applies to every robot in the solver, not just the main one.** All three
  `run*` functions loop `for(size_t i = 0; i < robots().size(); ++i)`. A closed-loop
  mode silently pulls `realRobots().robot(i)` for *every* $i$. If an observer pipeline
  populates only robot 0, robots $1..n$ get $\hat q = $ whatever `realRobots()` was
  last left holding — which after construction is a copy of the initial state.

- **Commanded torque follows the build state, not the integration state.** In the Tasks
  backend, after the switch returns:

  ```cpp
  for(auto * dyn : dynamicsConstraints_) {
    dyn->motionConstr().computeTorque(solver_.alphaDVec(), solver_.lambdaVec());
    rbd::vectorToParam(dyn->motionConstr().torque(), robot(dyn->robotIndex()).mbc().jointTorque);
  }
  ```

  `motionConstr()` was built at $q^{\text{qp}}_k$, so $\tau_k = \tau(q^{\text{qp}}_k,
  \alpha^{\text{qp}}_k, \ddot q^\star_k, \lambda^\star_k)$ in all modes. The TVM
  backend reads `tvm_robot.tau()->value()` inside `updateRobot`, same conclusion.

- **`incrementIterInSolver()` is called on the build state.** Trajectory tasks advance
  their internal clock during the overwritten window. Harmless, but it means task
  internal state is a function of $q^{\text{qp}}$.

- **The save/restore is a whole-`mbc` round trip.** `control_q_` / `control_alpha_` are
  `std::vector<std::vector<double>>` copies made every tick. Not free, but not on any
  hot path worth optimising.

- **`Joints` and `ObservedRobots` do not compose.** They are separate enum values, not
  flags. Encoder feedback with an observed floating base means using
  `ObservedRobots` with an `Encoder` observer in the pipeline — the pipeline is where
  the composition happens, not the enum.

- **Backends agree.** `TasksQPSolver` and `TVMQPSolver` have identical dispatch and
  identical save/overwrite/restore ordering. Minor divergence: TVM `continue`s on
  `nrDof() == 0` before the restore; Tasks restores unconditionally and guards only the
  integration. No semantic difference.