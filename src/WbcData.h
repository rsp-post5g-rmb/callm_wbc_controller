#pragma once

#include <array>
#include <cstddef>
#include <vector>

/** Command contract exchanged between the ROS2 client and CallmWbcController.
 *
 * The whole struct is serialized into a single flat std_msgs/Float64MultiArray
 * (see pack()/unpack()) so the ROS wire format stays trivial and mirrors the
 * explicit_compliance_controller reference. ALL six fields travel across the
 * wire on every message. Which ones are *actuated* is a controller concern, not
 * a transport concern:
 *
 *   ACTIVE (drive their targets every loop):
 *     - eef_pos, eef_quat  -> arm end-effector SurfaceTransformTask target
 *     - posture_arm        -> arm PostureTask target
 *     - velocity_base      -> base body velocity (integrated into the base task)
 *     - gripper_opening    -> Robotiq plugin command (RobotiqGripper::setOpening)
 *
 *   PLUMBED ONLY (received, stored, not yet actuated):
 *     - posture_base       -> single extension point in the controller
 *
 * Field layout inside the flat array (20 doubles total):
 *   [ 0.. 3) eef_pos         (x, y, z)                        world frame
 *   [ 3.. 7) eef_quat        (w, x, y, z)                     world frame
 *   [ 7..13) posture_arm     (6 UR5e joints, ref_joint_order)
 *   [13..16) posture_base    (base_x, base_y, base_yaw)
 *   [16..17) gripper_opening (0 = open, 1 = closed)           Robotiq convention
 *   [17..20) velocity_base   (vx, vy, wyaw)                   base body frame
 */
struct WbcData
{
  std::array<double, 3> eef_pos = {0.0, 0.0, 0.0};
  std::array<double, 4> eef_quat = {1.0, 0.0, 0.0, 0.0}; ///< (w, x, y, z)
  std::array<double, 6> posture_arm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> posture_base = {0.0, 0.0, 0.0}; ///< plumbed only (extension point)
  double gripper_opening = 0.0; ///< 0 = open, 1 = closed (Robotiq plugin convention)
  std::array<double, 3> velocity_base = {0.0, 0.0, 0.0}; ///< (vx, vy, wyaw) body frame

  // ---- Flat layout (offsets into the serialized array) ----------------------
  static constexpr std::size_t EEF_POS_OFFSET = 0;
  static constexpr std::size_t EEF_QUAT_OFFSET = 3;
  static constexpr std::size_t POSTURE_ARM_OFFSET = 7;
  static constexpr std::size_t POSTURE_BASE_OFFSET = 13;
  static constexpr std::size_t GRIPPER_OPENING_OFFSET = 16;
  static constexpr std::size_t VELOCITY_BASE_OFFSET = 17;
  static constexpr std::size_t SIZE = 20;

  /** Serialize into a flat vector of length SIZE. */
  std::vector<double> pack() const
  {
    std::vector<double> out;
    out.reserve(SIZE);
    out.insert(out.end(), eef_pos.begin(), eef_pos.end());
    out.insert(out.end(), eef_quat.begin(), eef_quat.end());
    out.insert(out.end(), posture_arm.begin(), posture_arm.end());
    out.insert(out.end(), posture_base.begin(), posture_base.end());
    out.push_back(gripper_opening);
    out.insert(out.end(), velocity_base.begin(), velocity_base.end());
    return out;
  }

  /** Deserialize from a flat vector. Returns false on a size mismatch. */
  static bool unpack(const std::vector<double> & data, WbcData & out)
  {
    if(data.size() != SIZE) { return false; }
    std::size_t o = 0;
    for(auto & v : out.eef_pos) { v = data[o++]; }
    for(auto & v : out.eef_quat) { v = data[o++]; }
    for(auto & v : out.posture_arm) { v = data[o++]; }
    for(auto & v : out.posture_base) { v = data[o++]; }
    out.gripper_opening = data[o++];
    for(auto & v : out.velocity_base) { v = data[o++]; }
    return true;
  }
};
