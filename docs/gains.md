# Per-Task Gains

Each of the four gain-controlled tasks (order `[ee, posture_arm, base, base_posture]`,
see [`tasks.md`](tasks.md)) has three live knobs, set via `WbcData`:

| Knob | `WbcData` field | meaning |
|---|---|---|
| weight $w$ | `task_weights` | task priority in the QP (the "mode") |
| stiffness $K$ | `task_stiffness` | proportional (tracking) gain |
| damping | `task_damping_ratio` | see below |

All use the sentinel `< 0` = "keep current / YAML default"; `>= 0` is adopted
(weights / stiffness clamped `>= 0`).

## Damping: ratio when $K>0$, absolute when $K=0$

The reference acceleration of a task is $a = -K\,e - D\,\dot e + a^{\mathrm{ff}}$. The
`task_damping_ratio` field sets $D$ with **one rule**:

$$
D \;=\;
\begin{cases}
2\,\zeta\,\sqrt{K} & K > 0 \quad(\text{field } = \zeta,\ \text{damping ratio};\ \zeta = 1 \Rightarrow \text{critical})\\[4pt]
\text{field} & K = 0 \quad(\text{field } = D,\ \textbf{absolute damping})
\end{cases}
$$

- **$K > 0$ (tracking tasks):** the field is a damping *ratio* $\zeta$, so you get
  critical damping by default ($\zeta = 1$) without hand-tuning $D$.
- **$K = 0$ (pure damper):** $2\zeta\sqrt{0} = 0$ would make the task inert, so the field
  is instead taken as the **absolute** damping $D$. The task then contributes
  $a = -D\,\dot e$ — an L2 penalty on joint velocity with **no** posture bias, i.e. a
  **velocity damper** (equivalent to a zero-gain posture / "damping" task).

Rationale: this keeps a single field and one consistent client rule, preserves
critical-by-default for the tracking tasks, and lets **any** task become a pure damper —
no separate task needed. Caveat: the meaning changes discontinuously at $K = 0$; that is
fine because $K = 0$ is a deliberate "damper mode" choice, not a value you sweep through.

## Example: immobilize the base (mink-style)

Use the **existing** base posture task (`triorbPostureTask_`) as a velocity damper —
drop the base transform authority and damp base velocity:

| field | ee | posture_arm | base | base_posture |
|---|---|---|---|---|
| `task_weights` | 1000 | 5 | **0** | **~1.5** |
| `task_stiffness` | 5 | 10 | — | **0** |
| `task_damping_ratio` | 1 | 1 | — | **$D>0$ (absolute)** |

Result: the EE task tracks the SE3 target, the arm posture is a weak retract bias, and
the base is held still by velocity damping (it still yields if the EE task truly needs it,
because the damper is a weighted soft cost). No extra task, no code beyond this rule.
