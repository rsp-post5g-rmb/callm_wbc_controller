# QP Task Formulation

Acceleration-level weighted QP over the stacked generalized acceleration of all
loaded robots. Solved each control step, then Euler-integrated.

## Variables

- Robots: UR5e (`UR5eFloatingBase`, index 0), TriOrb (index 1).
- $q_0 = (X_{\mathrm{fb}},\, q_{\mathrm{arm}})$, $\;X_{\mathrm{fb}}\in SE(3)$, $\;q_{\mathrm{arm}}\in\mathbb{R}^6$.
- $q_1 = (x,\,y,\,\theta)$ — TriOrb planar joints `base_x, base_y, base_yaw` (fixed root).
- Decision variable: $\ddot q = (\ddot q_0,\ \ddot q_1)$.

## Program

$$
\ddot q^\star \;=\; \arg\min_{\ddot q}\; \tfrac12 \sum_{i\in\mathcal T} w_i \,\big\lVert\, J_i\,\ddot q + \dot J_i\,\dot q - a_i \,\big\rVert^2
\qquad \text{s.t. } \ddot q \in \mathcal C
$$

Per-task reference acceleration (PD + feedforward):

$$
a_i \;=\; -K_i\,e_i \;-\; D_i\,\dot e_i \;+\; a_i^{\mathrm{ff}},
\qquad K_i = s_i I,\quad D_i = 2\,\zeta_i\sqrt{s_i}\, I
$$

Gains $(w_i,\, s_i,\, \zeta_i)$ = (weight, stiffness, damping ratio), set per task via
`WbcData` in order $[\,\text{ee},\ \text{posture\_arm},\ \text{base},\ \text{base\_posture}\,]$.

Integration: $\;\dot q \leftarrow \dot q + \ddot q^\star \Delta t,\quad q \leftarrow q \oplus \dot q\,\Delta t.$

## Tasks $\mathcal T$

**T1 — Arm end-effector** (`SurfaceTransformTask`, UR5e frame `Tool` = `tool0`, $F_T$)

$$
e_1 = \operatorname{err}_{SE(3)}\!\big(X_{F_T},\, X^\star_{\mathrm{ee}}\big)\in\mathbb{R}^6,
\qquad J_1 = J_{F_T},\qquad a_1^{\mathrm{ff}} = 0
$$

**T2 — Arm posture** (`PostureTask`, UR5e)

$$
e_2 = q_{\mathrm{arm}} - q^\star_{\mathrm{arm}}\in\mathbb{R}^6,
\qquad J_2 = S_{\mathrm{arm}}\ \text{(joint selection)}
$$

**T3 — Base transform** (`TransformTask`, TriOrb frame `base`, $F_B$)

$$
e_3 = \operatorname{err}_{SE(3)}\!\big(X_{F_B},\, X^\star_{\mathrm{base}}\big),
\qquad J_3 = J_{F_B},\qquad a_3^{\mathrm{ff}} \leftrightarrow \operatorname{refVelB} = V_b
$$

velocity command re-based off the current pose each tick: $\;X^\star_{\mathrm{base}} \leftarrow X_{F_B}\,\exp(\widehat{V_b}\,\Delta t),\ \ V_b = (v_x,\,v_y,\,0,\,0,\,0,\,\omega)$ — see [`base_velocity_target.md`](base_velocity_target.md).

**T4 — Base posture** (`PostureTask`, TriOrb)

$$
e_4 = q_1 - q^\star_1,\qquad q_1 = (x,\,y,\,\theta),\qquad J_4 = S_{\mathrm{base}}
$$

**T5 — Gripper posture** (`PostureTask`, only if `gripper.simulate: true`)

$$
e_5 = q_{\mathrm{grip}} - q^\star_{\mathrm{grip}},\qquad J_5 = S_{\mathrm{grip}}
$$

## Coupling contact (equality)

Rigid 6-DoF contact `triorb::ArmMount` $\leftrightarrow$ `ur5e::Base` — the UR5e floating
base $X_{\mathrm{fb}}$ tracks the TriOrb mount:

$$
J_c\,\ddot q + \dot J_c\,\dot q = 0
$$

(and, when simulated, `ur5e::Tool` $\leftrightarrow$ `gripper::Base`.)

## Constraint set $\mathcal C$

Kinematic limits (UR5e, TriOrb) as bounds on $\ddot q$:

$$
\underline q \le q \le \overline q,\qquad \lvert\dot q\rvert \le \dot q^{\max}
\;\Rightarrow\; \underline{\ddot q}(q,\dot q) \le \ddot q \le \overline{\ddot q}(q,\dot q)
$$

Collision avoidance (UR5e self-collision; arm-links $\leftrightarrow$ base) — velocity damper on
each monitored pair with signed distance $d(q)$:

$$
\dot d \;\ge\; -\,\xi\,\frac{d - d_s}{d_i - d_s},\qquad d \ge d_s
$$

$d_i$ = interaction distance, $d_s$ = safety distance, $\xi$ = damping.

## Outside the QP

- State grounding (pre-solve): $\;q_1 \leftarrow (x,y,\theta)_{\mathrm{SLAM}}$.
- Output demux (post-solve): arm $\to q_{\mathrm{arm}}$ (position); base $\to \dot q_1$
  (world $\to$ body) $\to$ base plugin; gripper opening $\to$ `RobotiqGripper::setOpening`.
