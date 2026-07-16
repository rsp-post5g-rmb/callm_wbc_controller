# Rotation Conventions (EE orientation)

## 1. Background: active vs frame rotations

Two conventions represent the same orientation as transposes of each other:

- **Active (Hamilton)** `R` — rotates points/vectors: `p_world = R · p_body`; the columns of
  `R` are the body axes in world. This is what ROS / tf2 / RViz / `geometry_msgs` quaternions
  and `Eigen::Quaterniond::toRotationMatrix()` give.
- **Frame** `E = Rᵀ` — rotates coordinates: `p_body = E · p_world`. **`sva::PTransformd` stores
  `E`**, so `X.rotation() == E` and the body's world orientation is `X.rotation().transpose()`.

Quaternion corollary: `quat(E) = quat(Rᵀ) = quat(R).conjugate()` — the sva-frame quaternion is
the **conjugate** of the Hamilton quaternion.

Failure mode: drop an **active** `R` into a slot that expects `E` (e.g. `PTransformd(R, t)`).
The represented world orientation becomes `Eᵀ = Rᵀ` — the **inverse**. A `+θ` becomes `−θ`.

## 2. The bug (this controller)

`WbcData.eef_quat` is a Hamilton quaternion. The command path built the target as

```cpp
sva::PTransformd eePose(q.toRotationMatrix(), pos);   // E := R  (WRONG: R is active)
```

so the EE task's world orientation was `Eᵀ = Rᵀ`: a commanded **+90°** about any axis was
realized as **−90°**. The measured echo had the mirror bug (`quat(E)` = conjugate of the Tool's
Hamilton orientation), so command↔measured round-tripped but the wire was non-standard.

## 3. The fix

Transpose at the wire↔pose boundary, and only there — via two helpers in
`src/CallmWbcController.cpp`:

```cpp
poseFromWire(wxyz, xyz) : Hamilton quat -> PTransformd     // E := R.transpose()
wireQuat(X)            : PTransformd    -> Hamilton quat    // quat(E.transpose())
```

`applyPendingCommand` and `collectMeasured` now use these, so `callm_wbc/command` and the
`callm_wbc/measured` echo are standard Hamilton and round-trip correctly. Internally everything
stays in sva convention. (This is the same transpose the `VisualOdometryObserver` applies to the
incoming VO pose.)

| EE target set via… | Convention | Status |
|---|---|---|
| ROS topic `callm_wbc/command` | Hamilton | ✅ fixed (`poseFromWire`) |
| RViz interactive marker (drag) | full pose, mc_rtc round-trips it | ✅ intuitive |
| RViz "EE target [world]" **numeric field** | raw sva frame quat `quat(E)` | ⚠️ conjugate |

## 4. RViz numeric-field workaround

The numeric quaternion in the `gui::Transform` "EE target [world]" field is mc_rtc's own
serialization of the `PTransformd` (raw `quat(E)`). The controller only supplies the getter/setter
`PTransformd` (`CallmWbcController.cpp:257`); it does not control how the widget encodes it, so §3
does not reach it. Dragging the marker (full pose) and the ROS topic need no workaround — only the
numeric field is the conjugate.

When typing into that field, enter the **conjugate**: keep `w`, negate `x, y, z`.

```
intended +90° about X (Hamilton):  wxyz =  0.707107,  0.707107, 0, 0
type into the numeric field:       wxyz =  0.707107, -0.707107, 0, 0
```

(For an intuitive numeric input in the GUI, add an `ArrayInput` routed through
`poseFromWire`/`wireQuat`.)
