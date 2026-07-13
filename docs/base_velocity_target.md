# Base Transform Target Update (velocity command)

How the base `TransformTask` (T3 in [`tasks.md`](tasks.md)) target is produced when the
base is commanded by **velocity** (`velocity_base`, active path).

## Setup

- $X_B$ — current base body pose, `robots("triorb").bodyPosW("base")`.
- $V_b = (v_x,\,v_y,\,\omega)$ — commanded body-frame velocity (`BASE_VELOCITY_KEY`).
- $X^\star$ — the base task target; the task also gets the feed-forward `refVelB` $= V_b$.
- $\Delta t$ — control timestep.

Planar advance operator $\oplus$ (yaw $\theta$ of the pose being advanced):

$$
X \oplus V_b\,\Delta t:\quad
\begin{aligned}
x &\mathrel{+}= (\cos\theta\, v_x - \sin\theta\, v_y)\,\Delta t\\
y &\mathrel{+}= (\sin\theta\, v_x + \cos\theta\, v_y)\,\Delta t\\
\theta &\mathrel{+}= \omega\,\Delta t
\end{aligned}
$$

## Update rules

**Old — accumulating (persistent target):**

$$
X^\star(k) \;=\; X^\star(k-1)\,\oplus\,V_b\,\Delta t
$$

The target integrates from its own previous value, independent of the measured base.
Re-anchored only in `reset()`.

**New — re-based (one-step-ahead carrot):**

$$
X^\star(k) \;=\; X_B(k)\,\oplus\,V_b\,\Delta t
$$

The target is re-derived from the **current** base pose every tick, so the position error
is bounded: $\lVert X^\star - X_B\rVert \approx V_b\,\Delta t$.

## Why the change

The accumulating rule fails in two ways:

1. **Conflict wind-up** (occurs even in pure sim). If the base cannot track the command
   because of conflicting objectives — e.g. the EE task holds the hand at a world pose and
   resists base motion through the arm↔base contact, or a collision / joint limit blocks the
   base — then $X^\star$ keeps advancing while $X_B$ lags, so $\lVert X^\star - X_B\rVert$
   grows **unbounded**. The base task then pushes ever harder (and can overpower other
   tasks), and **lurches** to catch up once the conflict clears.

2. **Episode-boundary lurch** (real hardware). On a SLAM origin re-zero, $X_B$ snaps back to
   the origin while $X^\star$ keeps its accumulated value → large error → lurch.

The re-based rule removes both: the target can never separate from the base by more than
$V_b\,\Delta t$, so there is no wind-up and no lurch. A SLAM re-zero is absorbed
automatically because $X^\star$ re-bases off the (re-zeroed) $X_B$.

## Trade-off

Re-basing makes the base a **pure velocity follower** (motion comes from the `refVelB`
feed-forward; the near-zero position error means the stiffness term barely acts):

- No **catch-up**: if the base lags under conflict it does not make up the lost distance —
  it just tracks the commanded velocity from wherever it is. Distance-over-time is not
  guaranteed (acceptable for closed-loop / policy control; not for open-loop distance goals).

## Sim ↔ real consistency

`integrateBaseVelocity()` is a **single unconditional code path** — no `sim`/`real`
branch. Only the *source* of $X_B$ differs, and the behavior is identical either way:

| Env | $X_B$ (`bodyPosW("base")`) is | 
| --- | --- |
| Sim (no SLAM) | the QP-integrated base pose |
| Real (SLAM) | the SLAM-grounded base pose (set in `applyBaseState()` pre-solve) |

What you observe/tune in the ticker matches hardware.

## Alternative (not implemented): leash

Keep the accumulating rule but clamp the target within a radius $R$ of $X_B$:
if $\lVert X^\star - X_B\rVert > R$, project $X^\star$ back onto the $R$-ball. This bounds the
wind-up to $R$ while preserving short-range catch-up — a middle ground if position
authority is wanted without unbounded drift.
