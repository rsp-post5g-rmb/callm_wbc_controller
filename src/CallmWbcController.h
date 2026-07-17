#pragma once

#include <mc_control/mc_controller.h>

#include <mc_solver/QPSolver.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/TransformTask.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "WbcData.h"
#include "api.h"

/** Whole-body controller for a UR5e arm mounted on a TriOrb omnidirectional base.
 *
 * A single QP coordinates the arm joints and the base planar motion jointly:
 *   - robot 0: callm -- the TriOrb base, the UR5e arm and (optionally) the Robotiq
 *              gripper fused into ONE robot module by mc_rbdyn::RobotModule::connect
 *   - robot 1: env/ground (visualization only)
 *
 * The arm is bolted to the TriOrb `mount` link, and the gripper to the arm's `tool0`, by
 * fixed CONNECT JOINTS, not by contacts: robotModules() merges the modules before
 * MCController loads them, so the QP sees a single kinematic tree from `odom` through
 * base_x/base_y/base_yaw to the fingertips. When the end-effector task pulls the hand the
 * solver distributes the motion across the arm joints and the base degrees of freedom.
 * That is the whole-body coordination, with no explicit inverse kinematics.
 *
 * The attachment is therefore structural: there is no coupling constraint to maintain, no
 * contact frame to capture, and the QP runs with NO contacts at all. See
 * docs/connect_migration.md for the migration rationale and the API evidence.
 *
 * Task naming follows what each task acts on:
 *   - callm_*  : the whole merged robot (callm_ee -- the Tool target is reached with the
 *                arm and the base together)
 *   - ur5e_*   : the 6 arm joints only (ur5e_posture)
 *   - triorb_* : the 3 base joints only (triorb_base, triorb_posture)
 *   - gripper_ : the gripper's own joints only (gripper_posture), matching the prefix
 *                they carry in the merged robot
 *
 * Commands arrive from a ROS2 client as a WbcData message (see WbcData.h). The ROS
 * node spins on its own thread and hands the latest command to the control loop
 * through a mutex-guarded buffer; run() applies it on the control thread (writing the
 * datastore + tasks + gripper), so no mc_rtc object is ever touched off the loop.
 */
struct CallmWbcController_DLLAPI CallmWbcController : public mc_control::MCController
{
  CallmWbcController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  ~CallmWbcController() override;

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  /** Datastore keys an external commander (GUI / ROS) can read/assign. Named after the part
   * each one drives, like the tasks they feed: callm_ = whole robot, ur5e_ = arm joints,
   * triorb_ = base joints. */
  static constexpr auto CALLM_EE_TARGET_KEY = "CallmWbcController::callmEeTarget"; ///< sva::PTransformd (world)
  /// sva::PTransformd (world), inactive path
  static constexpr auto TRIORB_BASE_TARGET_KEY = "CallmWbcController::triorbBaseTarget";
  /// Eigen::Vector3d (vx, vy, wyaw), base body frame
  static constexpr auto TRIORB_BASE_VELOCITY_KEY = "CallmWbcController::triorbBaseVelocity";
  static constexpr auto UR5E_POSTURE_KEY = "CallmWbcController::ur5ePosture"; ///< std::vector<double> (6 arm joints)
  static constexpr auto TRIORB_POSTURE_KEY = "CallmWbcController::triorbPosture"; ///< std::vector<double> (3 base joints)
  static constexpr auto GRIPPER_OPENING_KEY = "CallmWbcController::gripperOpening"; ///< double (0 open, 1 closed)
  // Per-task gains, stored as std::vector<double> of size 4 (NOT Eigen::Vector4d, which is
  // over-aligned and unsafe in the type-erased DataStore).
  // Order: [callm_ee, ur5e_posture, triorb_base, triorb_posture].
  static constexpr auto TASK_WEIGHTS_KEY = "CallmWbcController::taskWeights";
  static constexpr auto TASK_STIFFNESS_KEY = "CallmWbcController::taskStiffness";
  static constexpr auto TASK_DAMPING_KEY = "CallmWbcController::taskDampingRatio"; ///< zeta (damping = 2*zeta*sqrt(stiff))

private:
  /** Assemble the robot-module vector (UR5e + TriOrb + ground [+ Robotiq gripper]).
   * Runs in the member-initializer list, so it reads the gripper options straight
   * from the controller configuration. */
  static std::vector<mc_rbdyn::RobotModulePtr> robotModules(mc_rbdyn::RobotModulePtr rm,
                                                            const mc_rtc::Configuration & config);

  /** Build the GUI elements and the datastore entries bound to the commanded targets. */
  void setupTargetsIO();

  /** Copy the ROS-buffered command (if any) into the datastore, on the control thread. */
  void applyPendingCommand();

  /** Push the datastore-held commands into the live tasks / gripper. */
  void applyCommandsToTasks();

  /** Advance the base transform-task target by the commanded body velocity (one dt step). */
  void integrateBaseVelocity();

  /** Hand the QP-realized base velocity (body frame) to the TriorbBasePlugin, after the QP. */
  void exportBaseVelocity();

  /** True when the VisualOdometryObserver has a fresh base fix (datastore VO::isAlive). */
  bool voAlive() const;

  /** True when the base task/export should reference the MEASURED base pose (realRobots):
   *  only under a closed-loop feedback mode AND a fresh VO fix. Under open-loop (none/joints)
   *  the control base is never grounded to realRobots, so VO must not move the control base. */
  bool useMeasuredBase() const;

  // ---- ROS2 interface (own context + executor + spin thread) ----------------
  void setupRos();
  void stopRos();
  void handleCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publishMeasured();
  WbcData collectMeasured() const;

  // WBC tasks. All of them address the merged robot (index 0); the posture tasks are
  // restricted to disjoint joint sets with selectActiveJoints, and are renamed because
  // PostureTask names itself after the robot. Joint limits for every joint come from
  // mc_rtc's built-in robot-0 kinematicsConstraint, so there are no per-part
  // KinematicsConstraints. The arm posture task is MCController's built-in `postureTask`,
  // renamed to "ur5e_posture" and restricted to the arm in the constructor.
  std::shared_ptr<mc_tasks::SurfaceTransformTask> callmEeTask_; ///< "callm_ee": Tool target, whole robot
  std::shared_ptr<mc_tasks::TransformTask> triorbBaseTask_; ///< "triorb_base": base body pose
  std::shared_ptr<mc_tasks::PostureTask> triorbPostureTask_; ///< "triorb_posture": base joints only
  std::shared_ptr<mc_tasks::PostureTask> gripperPostureTask_; ///< "gripper_posture": gripper joints only

  // Task gains / weights (loaded from the controller configuration)
  double callmEeStiffness_ = 5.0;
  double callmEeWeight_ = 1000.0;
  double triorbBaseStiffness_ = 2.0;
  double triorbBaseWeight_ = 1000.0;
  double ur5ePostureStiffness_ = 10.0;
  double ur5ePostureWeight_ = 5.0;
  double triorbPostureStiffness_ = 1.0;
  double triorbPostureWeight_ = 1.0;
  double gripperPostureStiffness_ = 1.0;
  double gripperPostureWeight_ = 1.0;

  // Geometry of the TriOrb description (world Z of the relevant links at the home pose).
  // base link sits at z = 0.30 (see mc_triorb_description/urdf/triorb.urdf). The mount
  // link pose (z = 0.60, Rz(-90deg)) is baked into the merged module by connect(), so it
  // is intentionally not duplicated as a constant here.
  double baseHeight_ = 0.30;

  // Base command routing: "velocity" (active, from velocity_base) or "pose" (inactive path).
  std::string baseCommandMode_ = "velocity";
  sva::PTransformd triorbBaseTargetPose_ = sva::PTransformd::Identity(); ///< integrated base setpoint

  // Robotiq gripper. Two independent switches:
  //  - gripperEnabled_ : the gripper MODEL is connected into the merged robot (sim/viz).
  //  - gripperCommandEnabled_ : forward gripper_opening to the mc_robotiq plugin, which
  //    talks to the physical gripper over a socket and moves no model joints.
  // The command path does NOT require the model, so the real gripper can be driven with
  // gripper.simulate: false.
  bool gripperEnabled_ = true; ///< set from gripperJointNames_ being non-empty (model connected in)
  bool gripperCommandEnabled_ = true; ///< forward opening to RobotiqGripper::setOpening
  std::string gripperSetOpeningCall_ = "RobotiqGripper::setOpening"; ///< mc_robotiq datastore call

  // Joint orders (verified against the module ref_joint_order). The arm connect() is given
  // an empty prefix, so the arm and base joints keep these names in the merged robot,
  // whose ref_joint_order is baseJointNames_ followed by armJointNames_ (then the
  // gripper's, when connected).
  std::vector<std::string> armJointNames_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                             "wrist_1_joint",       "wrist_2_joint",       "wrist_3_joint"};
  std::vector<std::string> baseJointNames_ = {"base_x", "base_y", "base_yaw"};
  /// Actuated joints of the connected gripper, derived from the merged model in the
  /// constructor (everything actuated that is neither an arm nor a base joint). They carry
  /// the "gripper_" prefix that robotModules() gave the gripper connect.
  std::vector<std::string> gripperJointNames_;

  // ROS2 plumbing (mirrors explicit_compliance_controller).
  std::string rosNodeName_ = "callm_wbc_controller";
  std::string commandTopic_ = "callm_wbc/command";
  std::string measuredTopic_ = "callm_wbc/measured";
  bool publishMeasured_ = true;
  std::size_t publishDecimation_ = 10;
  std::size_t runCounter_ = 0;

  std::shared_ptr<rclcpp::Node> rosNode_;
  rclcpp::Context::SharedPtr rosContext_;
  rclcpp::CallbackGroup::SharedPtr rosCallbackGroup_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr commandSubscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr measuredPublisher_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor_;
  std::thread rosSpinThread_;

  // Cross-thread command hand-off (ROS spin thread -> control thread).
  mutable std::mutex commandMutex_;
  WbcData commandedData_;
  bool hasPendingCommand_ = false;

  // ---- Base localization + base-velocity export -----------------------------
  // The base is velocity-controlled hardware. Its measured *state* is grounded in
  // realRobots() by the VisualOdometryObserver (see docs/observer.md); the controller
  // consumes that estimate (re-base anchor / export yaw when VO is alive, and via the
  // QP feedback mode). The QP-realized base velocity is exported to the TriorbBasePlugin
  // (Triorb::cmd_velocity) as the *command*.
  std::string triorbCmdKey_ = "Triorb::cmd_velocity"; ///< TriorbBasePlugin body-velocity input

  // QP feedback mode: how the control robots consume realRobots() each tick. Set via the
  // `feedback` config key (none|joints|joints_velocity|observed|observed_real). Default
  // open-loop so pure sim (no VO publisher) keeps working; use `observed` on hardware.
  mc_solver::FeedbackType feedbackType_ = mc_solver::FeedbackType::None;

  bool weightsDegenerateWarned_ = false; ///< one-shot warn on a degenerate (no arm authority) weight set

  bool ioReady_ = false; ///< guards one-time GUI/datastore setup across resets
};
