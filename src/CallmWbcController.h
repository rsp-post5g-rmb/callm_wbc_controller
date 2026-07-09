#pragma once

#include <mc_control/mc_controller.h>

#include <mc_solver/KinematicsConstraint.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/TransformTask.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
 *   - robot 0: UR5e (floating base, `MainRobot: UR5eFloatingBase`)
 *   - robot 1: triorb (the TriOrb base)
 *   - robot 2: env/ground (visualization only)
 *   - robot 3: robotiq_2f_85_gripper (rigidly bolted onto the UR5e tool)
 *
 * The UR5e floating base is rigidly attached to the TriOrb `mount` link through a
 * Base<->Base contact, so when the QP moves the base the arm follows, and when the
 * end-effector task pulls the hand the solver distributes the motion across both the
 * arm joints and the base degrees of freedom. That is the whole-body coordination,
 * with no explicit inverse kinematics.
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

  /** Datastore keys an external commander (GUI / ROS) can read/assign. */
  static constexpr auto EE_TARGET_KEY = "CallmWbcController::eeTarget"; ///< sva::PTransformd (world)
  static constexpr auto BASE_TARGET_KEY = "CallmWbcController::baseTarget"; ///< sva::PTransformd (world), inactive path
  static constexpr auto BASE_VELOCITY_KEY = "CallmWbcController::baseVelocity"; ///< Eigen::Vector3d (vx, vy, wyaw) body
  static constexpr auto ARM_POSTURE_KEY = "CallmWbcController::armPosture"; ///< std::vector<double> (6 UR5e joints)
  static constexpr auto BASE_POSTURE_KEY = "CallmWbcController::basePosture"; ///< std::vector<double> (3), plumbed only
  static constexpr auto GRIPPER_OPENING_KEY = "CallmWbcController::gripperOpening"; ///< double (0 open, 1 closed)
  // Per-task gains as Eigen::Vector4d, order [ee, posture_arm, base, base_posture].
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

  /** Fold the latest SLAM pose into the TriOrb base joints (measured state, before the QP). */
  void applyBaseState();

  /** Hand the QP-realized base velocity (body frame) to the TriorbBasePlugin, after the QP. */
  void exportBaseVelocity();

  // ---- ROS2 interface (own context + executor + spin thread) ----------------
  void setupRos();
  void stopRos();
  void handleCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void handleSlamPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishMeasured();
  WbcData collectMeasured() const;

  // WBC tasks / constraints
  std::shared_ptr<mc_tasks::SurfaceTransformTask> eeTask_; ///< UR5e end-effector (Tool surface)
  std::shared_ptr<mc_tasks::TransformTask> baseTask_; ///< TriOrb base body pose
  std::shared_ptr<mc_tasks::PostureTask> triorbPostureTask_; ///< regularizes the base joints
  std::unique_ptr<mc_solver::KinematicsConstraint> triorbKinematics_; ///< base joint limits
  std::shared_ptr<mc_tasks::PostureTask> gripperPostureTask_; ///< holds the gripper knuckle joints
  std::unique_ptr<mc_solver::KinematicsConstraint> gripperKinematics_; ///< gripper joint limits

  // Task gains / weights (loaded from the controller configuration)
  double eeStiffness_ = 5.0;
  double eeWeight_ = 1000.0;
  double baseStiffness_ = 2.0;
  double baseWeight_ = 1000.0;
  double ur5ePostureStiffness_ = 10.0;
  double ur5ePostureWeight_ = 5.0;
  double triorbPostureStiffness_ = 1.0;
  double triorbPostureWeight_ = 1.0;
  double gripperPostureStiffness_ = 1.0;
  double gripperPostureWeight_ = 1.0;

  // Geometry of the TriOrb description (world Z of the relevant links at the home pose).
  // base link sits at z = 0.30, mount link at z = 0.60 (see mc_triorb_description/urdf/triorb.urdf).
  double baseHeight_ = 0.30;
  double mountHeight_ = 0.60;

  // Names of the runtime-added attachment surface on the TriOrb mount link.
  std::string armMountSurface_ = "ArmMount";

  // Base command routing: "velocity" (active, from velocity_base) or "pose" (inactive path).
  std::string baseCommandMode_ = "velocity";
  sva::PTransformd baseTargetPose_ = sva::PTransformd::Identity(); ///< integrated base setpoint

  // Robotiq gripper (separate loaded robot, attached to the UR5e Tool surface).
  bool gripperEnabled_ = true;
  std::string gripperModule_ = "Robotiq2f85Gripper"; ///< RobotLoader name (mc_robot_tools)
  std::string gripperRobot_ = "robotiq_2f_85_gripper"; ///< robot name = module name
  std::string gripperBaseSurface_ = "Base"; ///< planar surface on the gripper base link
  std::string gripperBaseLink_ = "robotiq_85_base_link"; ///< base link for the runtime attachment surface
  std::string gripperSetOpeningCall_ = "RobotiqGripper::setOpening"; ///< mc_robotiq datastore call

  // Joint orders (verified against the module ref_joint_order).
  std::vector<std::string> armJointNames_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                             "wrist_1_joint",       "wrist_2_joint",       "wrist_3_joint"};
  std::vector<std::string> baseJointNames_ = {"base_x", "base_y", "base_yaw"};

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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slamSubscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr measuredPublisher_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor_;
  std::thread rosSpinThread_;

  // Cross-thread command hand-off (ROS spin thread -> control thread).
  mutable std::mutex commandMutex_;
  WbcData commandedData_;
  bool hasPendingCommand_ = false;

  // ---- Base localization from SLAM (ROS) ------------------------------------
  // The base is velocity-controlled hardware; its *state* is grounded every tick
  // from /robot_pose_slam (drift-corrected), the base analogue of mc_rtde feeding
  // the arm encoders back. The QP-realized base velocity is exported to the
  // TriorbBasePlugin (Triorb::cmd_velocity) as the *command*.
  bool baseStateFromSlam_ = true;
  std::string slamTopic_ = "/robot_pose_slam";
  std::string slamFrameMode_ = "capture_offset"; ///< "capture_offset" (default) or "world" (raw)
  double slamTimeout_ = 0.5; ///< s; warn (once) and hold-last beyond this
  std::string triorbCmdKey_ = "Triorb::cmd_velocity"; ///< TriorbBasePlugin body-velocity input

  struct SlamPose
  {
    double x = 0.0, y = 0.0, yaw = 0.0;
    std::chrono::steady_clock::time_point recv{};
    bool valid = false;
  };
  mutable std::mutex slamMutex_;
  SlamPose slamPose_; ///< latest SLAM pose (spin thread -> control thread)
  // Capture-offset state (control-thread only): world = R(-offYaw)*(slam - off).
  bool slamOffsetCaptured_ = false;
  double slamOffX_ = 0.0, slamOffY_ = 0.0, slamOffYaw_ = 0.0;
  bool slamStaleWarned_ = false;

  bool weightsDegenerateWarned_ = false; ///< one-shot warn on a degenerate (no arm authority) weight set

  bool ioReady_ = false; ///< guards one-time GUI/datastore setup across resets
};
