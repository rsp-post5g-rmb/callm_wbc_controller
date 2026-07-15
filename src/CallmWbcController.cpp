#include "CallmWbcController.h"

#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/NumberSlider.h>
#include <mc_rtc/gui/Transform.h>
#include <mc_rtc/logging.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>

namespace
{
/** Clamp every element to >= 0 (weights/stiffness/damping-ratio are non-negative). */
std::vector<double> clampNonNeg(std::vector<double> v)
{
  for(auto & x : v) { x = std::max(0.0, x); }
  return v;
}

/** Map the `feedback` config string to the QP FeedbackType (how robots() consume realRobots()). */
mc_solver::FeedbackType parseFeedback(const std::string & s)
{
  if(s == "none" || s == "open" || s == "openloop") { return mc_solver::FeedbackType::None; }
  if(s == "joints") { return mc_solver::FeedbackType::Joints; }
  if(s == "joints_velocity" || s == "jointsWVelocity") { return mc_solver::FeedbackType::JointsWVelocity; }
  if(s == "observed" || s == "closed" || s == "closedloop") { return mc_solver::FeedbackType::ObservedRobots; }
  if(s == "observed_real" || s == "closedLoopIntegrateReal") { return mc_solver::FeedbackType::ClosedLoopIntegrateReal; }
  mc_rtc::log::warning("[CallmWbcController] unknown feedback '{}'; using open-loop (none)", s);
  return mc_solver::FeedbackType::None;
}
} // namespace

std::vector<mc_rbdyn::RobotModulePtr> CallmWbcController::robotModules(mc_rbdyn::RobotModulePtr rm,
                                                                       const mc_rtc::Configuration & config)
{
  // robot 0: UR5e (the MainRobot, UR5eFloatingBase) ; 1: triorb ; 2: env/ground
  std::vector<mc_rbdyn::RobotModulePtr> modules;
  modules.push_back(rm);
  modules.push_back(mc_rbdyn::RobotLoader::get_robot_module("triorb"));
  modules.push_back(mc_rbdyn::RobotLoader::get_robot_module("env/ground"));

  // robot 3: Robotiq gripper (optional). It is a ConnectableRobotModule from
  // mc_robot_tools ("Robotiq2f85Gripper" / "Robotiq2f140Gripper"), loaded here as a
  // separate robot and bolted onto the UR5e tool in reset(). If the module is not on
  // the path we log and continue without it rather than aborting construction.
  //
  // `gripper.simulate` controls loading this MODEL only; commanding the real gripper
  // goes through the mc_robotiq plugin (gripper.command) and is independent of it. Set
  // simulate=false to run without the mc_robot_tools gripper model entirely.
  bool gripperSimulate = true;
  std::string gripperModule = "Robotiq2f85Gripper";
  if(config.has("gripper"))
  {
    auto g = config("gripper");
    g("enable", gripperSimulate); // legacy alias
    g("simulate", gripperSimulate);
    g("model", gripperModule);
  }
  if(gripperSimulate)
  {
    try
    {
      auto gm = mc_rbdyn::RobotLoader::get_robot_module(gripperModule);
      if(gm) { modules.push_back(gm); }
      else { mc_rtc::log::warning("[CallmWbcController] Gripper module '{}' not found; running without it", gripperModule); }
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::warning("[CallmWbcController] Could not load gripper module '{}': {}. Running without it",
                           gripperModule, e.what());
    }
  }
  return modules;
}

CallmWbcController::CallmWbcController(mc_rbdyn::RobotModulePtr rm,
                                      double dt,
                                      const mc_rtc::Configuration & config)
: mc_control::MCController(robotModules(rm, config), dt)
{
  // ---- Configurable task gains / weights -----------------------------------
  auto loadGains = [&](const std::string & key, double & stiffness, double & weight)
  {
    if(config.has(key))
    {
      auto c = config(key);
      c("stiffness", stiffness);
      c("weight", weight);
    }
  };
  loadGains("ee_task", eeStiffness_, eeWeight_);
  loadGains("base_task", baseStiffness_, baseWeight_);
  loadGains("ur5e_posture", ur5ePostureStiffness_, ur5ePostureWeight_);
  loadGains("triorb_posture", triorbPostureStiffness_, triorbPostureWeight_);
  loadGains("gripper_posture", gripperPostureStiffness_, gripperPostureWeight_);

  // ---- Base command routing (velocity is the active path) ------------------
  if(config.has("base_command")) { config("base_command")("mode", baseCommandMode_); }

  // ---- Gripper naming + command switch -------------------------------------
  if(config.has("gripper"))
  {
    auto g = config("gripper");
    g("model", gripperModule_);
    g("set_opening_call", gripperSetOpeningCall_);
    g("command", gripperCommandEnabled_); // forward opening to the mc_robotiq plugin (real gripper)
  }
  const bool is140 = gripperModule_.find("140") != std::string::npos;
  gripperRobot_ = is140 ? "robotiq_2f_140_gripper" : "robotiq_2f_85_gripper";
  gripperBaseLink_ = is140 ? "robotiq_140_base_link" : "robotiq_85_base_link";
  gripperEnabled_ = robots().hasRobot(gripperRobot_); // model loaded (independent of command)

  // Ensure the gripper has the attachment surface used by the Tool<->Base contact in
  // reset(). mc_robot_tools ships it in an RSDF, but that RSDF is not always installed
  // where mc_rtc looks; add an equivalent planar surface at runtime if it is missing
  // (same approach as the TriOrb ArmMount surface). Footprint matches the RSDF (+-0.0375).
  if(gripperEnabled_ && !robot(gripperRobot_).hasSurface(gripperBaseSurface_))
  {
    std::vector<std::pair<double, double>> gPts = {
        {-0.0375, -0.0375}, {0.0375, -0.0375}, {0.0375, 0.0375}, {-0.0375, 0.0375}};
    robot(gripperRobot_)
        .addSurface(std::make_shared<mc_rbdyn::PlanarSurface>(gripperBaseSurface_, gripperBaseLink_,
                                                              sva::PTransformd::Identity(), "plastic", gPts));
    mc_rtc::log::info("[CallmWbcController] added runtime '{}' surface on {}::{}", gripperBaseSurface_, gripperRobot_,
                      gripperBaseLink_);
  }

  // ---- ROS interface options -----------------------------------------------
  if(config.has("ros"))
  {
    auto r = config("ros");
    r("node_name", rosNodeName_);
    r("command_topic", commandTopic_);
    r("measured_topic", measuredTopic_);
    r("publish_measured", publishMeasured_);
    r("publish_decimation", publishDecimation_);
  }

  // ---- Base-velocity export key + QP feedback mode --------------------------
  // Base *state* now comes from the VisualOdometryObserver (realRobots); the controller
  // only needs the export key here and the feedback mode. See docs/observer.md.
  if(config.has("base_state")) { config("base_state")("command_key", triorbCmdKey_); }
  std::string feedback = "none";
  config("feedback", feedback);
  feedbackType_ = parseFeedback(feedback);

  // ---- Constraints + main-robot posture ------------------------------------
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint); // UR5e (robot 0) joint limits
  solver().addConstraintSet(selfCollisionConstraint); // UR5e self-collisions
  solver().addTask(postureTask); // UR5e posture (robot 0)
  solver().setContacts({}); // start with no contacts; couplings are added in reset()

  // ---- Arm <-> base attachment surface -------------------------------------
  // The TriOrb module ships no RSDF surfaces, so we declare, at runtime, a planar
  // surface on its `mount` link that mirrors the UR5e "Base" surface. The arm's
  // floating base is contacted onto this surface in reset(). The points match the
  // UR5e base footprint (see mc_ur5e_description/rsdf/ur5e/base_link.rsdf).
  std::vector<std::pair<double, double>> mountPoints = {
      {-0.0745, -0.0745}, {0.0745, -0.0745}, {0.0745, 0.0745}, {-0.0745, 0.0745}};
  robot("triorb").addSurface(std::make_shared<mc_rbdyn::PlanarSurface>(
      armMountSurface_, "mount", sva::PTransformd::Identity(), "plastic", mountPoints));

  // ---- Arm <-> base collision avoidance ------------------------------------
  // mc_rtc auto-builds an sch::S_Box collision convex named "base" for the TriOrb
  // base link directly from the URDF <box> collision primitive, so no extra hull
  // file is needed. We guard the distal arm links against that box. base_link and
  // shoulder_link are intentionally excluded: the arm base is rigidly mounted on
  // top of the base, so they sit permanently against it by design.
  bool enableArmBaseCollision = true;
  double ciDist = 0.05; // interaction distance: avoidance starts engaging here
  double csDist = 0.02; // safety distance: hard lower bound on separation
  std::vector<std::string> armLinks = {"forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"};
  if(config.has("arm_base_collision"))
  {
    auto c = config("arm_base_collision");
    c("enable", enableArmBaseCollision);
    c("iDist", ciDist);
    c("sDist", csDist);
    c("arm_links", armLinks);
  }
  if(enableArmBaseCollision)
  {
    std::vector<mc_rbdyn::Collision> armBaseCollisions;
    for(const auto & link : armLinks) { armBaseCollisions.push_back({link, "base", ciDist, csDist, 0.0}); }
    addCollisions("ur5e", "triorb", armBaseCollisions);
  }

  setupTargetsIO();
  setupRos();

  mc_rtc::log::success("CallmWbcController init done (gripper model: {}, command: {})",
                       gripperEnabled_ ? gripperRobot_ : "not simulated", gripperCommandEnabled_ ? "on" : "off");
}

CallmWbcController::~CallmWbcController()
{
  stopRos();
}

void CallmWbcController::setupTargetsIO()
{
  if(ioReady_) { return; }
  ioReady_ = true;

  // Datastore is the single source of truth for the commanded targets. The GUI and
  // the ROS handler (via applyPendingCommand) read/write these keys; run() then pushes
  // them onto the live tasks.
  datastore().make<sva::PTransformd>(EE_TARGET_KEY, sva::PTransformd::Identity());
  datastore().make<sva::PTransformd>(BASE_TARGET_KEY, sva::PTransformd(Eigen::Vector3d(0.0, 0.0, baseHeight_)));
  datastore().make<Eigen::Vector3d>(BASE_VELOCITY_KEY, Eigen::Vector3d::Zero());
  datastore().make<std::vector<double>>(ARM_POSTURE_KEY, std::vector<double>(armJointNames_.size(), 0.0));
  datastore().make<std::vector<double>>(BASE_POSTURE_KEY, std::vector<double>(baseJointNames_.size(), 0.0));
  datastore().make<double>(GRIPPER_OPENING_KEY, 0.0);
  // Per-task gains, order [ee, posture_arm, base, base_posture], seeded from the YAML
  // defaults (damping ratio = 1 => critically damped). A client picks a "mode" via the
  // weights and shapes compliance via stiffness/damping. See applyPendingCommand for
  // the <0 = keep-default sentinel.
  datastore().make<std::vector<double>>(
      TASK_WEIGHTS_KEY, std::vector<double>{eeWeight_, ur5ePostureWeight_, baseWeight_, triorbPostureWeight_});
  datastore().make<std::vector<double>>(
      TASK_STIFFNESS_KEY,
      std::vector<double>{eeStiffness_, ur5ePostureStiffness_, baseStiffness_, triorbPostureStiffness_});
  datastore().make<std::vector<double>>(TASK_DAMPING_KEY, std::vector<double>{1.0, 1.0, 1.0, 1.0});

  gui()->addElement(
      {"CallmWbc"},
      mc_rtc::gui::Transform(
          "EE target [world]", [this]() -> sva::PTransformd
          { return datastore().get<sva::PTransformd>(EE_TARGET_KEY); },
          [this](const sva::PTransformd & p) { datastore().assign<sva::PTransformd>(EE_TARGET_KEY, p); }),
      mc_rtc::gui::ArrayInput(
          "Base velocity [vx, vy, wyaw] (body)", {"vx", "vy", "wyaw"},
          [this]() -> Eigen::Vector3d { return datastore().get<Eigen::Vector3d>(BASE_VELOCITY_KEY); },
          [this](const Eigen::Vector3d & v) { datastore().assign<Eigen::Vector3d>(BASE_VELOCITY_KEY, v); }),
      mc_rtc::gui::ArrayInput(
          "Arm posture [rad]", armJointNames_,
          [this]() -> std::vector<double> { return datastore().get<std::vector<double>>(ARM_POSTURE_KEY); },
          [this](const std::vector<double> & q) { datastore().assign<std::vector<double>>(ARM_POSTURE_KEY, q); }),
      mc_rtc::gui::NumberSlider(
          "Gripper opening (0=open, 1=closed)", [this]() { return datastore().get<double>(GRIPPER_OPENING_KEY); },
          [this](double v) { datastore().assign<double>(GRIPPER_OPENING_KEY, std::max(0.0, std::min(1.0, v))); }, 0.0,
          1.0),
      mc_rtc::gui::ArrayInput(
          "Task weights [ee, arm, base, base_post]", {"ee", "arm", "base", "base_post"},
          [this]() -> std::vector<double> { return datastore().get<std::vector<double>>(TASK_WEIGHTS_KEY); },
          [this](const std::vector<double> & v)
          { datastore().assign<std::vector<double>>(TASK_WEIGHTS_KEY, clampNonNeg(v)); }),
      mc_rtc::gui::ArrayInput(
          "Task stiffness [ee, arm, base, base_post]", {"ee", "arm", "base", "base_post"},
          [this]() -> std::vector<double> { return datastore().get<std::vector<double>>(TASK_STIFFNESS_KEY); },
          [this](const std::vector<double> & v)
          { datastore().assign<std::vector<double>>(TASK_STIFFNESS_KEY, clampNonNeg(v)); }),
      mc_rtc::gui::ArrayInput(
          "Task damping ratio [ee, arm, base, base_post]", {"ee", "arm", "base", "base_post"},
          [this]() -> std::vector<double> { return datastore().get<std::vector<double>>(TASK_DAMPING_KEY); },
          [this](const std::vector<double> & v)
          { datastore().assign<std::vector<double>>(TASK_DAMPING_KEY, clampNonNeg(v)); }));

  // Inactive/overridable base-pose command path (used only when base_command.mode = "pose").
  gui()->addElement(
      {"CallmWbc", "Base pose (inactive path)"},
      mc_rtc::gui::Transform(
          "Base target [world]", [this]() -> sva::PTransformd
          { return datastore().get<sva::PTransformd>(BASE_TARGET_KEY); },
          [this](const sva::PTransformd & p) { datastore().assign<sva::PTransformd>(BASE_TARGET_KEY, p); }),
      mc_rtc::gui::Button("Sync targets to current robot",
                          [this]()
                          {
                            datastore().assign<sva::PTransformd>(EE_TARGET_KEY,
                                                                 robots().robot("ur5e").surfacePose("Tool"));
                            datastore().assign<sva::PTransformd>(BASE_TARGET_KEY,
                                                                 robots().robot("triorb").bodyPosW("base"));
                          }));
}

bool CallmWbcController::run()
{
  applyPendingCommand(); // ROS-buffered command -> datastore (control thread)
  applyCommandsToTasks(); // datastore -> live tasks / gripper
  // QP solves + euler-integrates. feedbackType_ selects how robots() consume realRobots()
  // (VO base + Encoder joints), grounded by the observer pipeline before this runs.
  bool ok = mc_control::MCController::run(feedbackType_);
  exportBaseVelocity(); // QP-realized base velocity (body frame) -> TriorbBasePlugin
  if(publishMeasured_ && measuredPublisher_ && (runCounter_ % publishDecimation_ == 0)) { publishMeasured(); }
  ++runCounter_;
  return ok;
}

void CallmWbcController::applyPendingCommand()
{
  WbcData cmd;
  {
    // try_lock: never block the (possibly SCHED_DEADLINE) control loop on the ROS
    // spin thread. If the callback holds the lock this tick, the command stays
    // pending and is applied next tick (latest-wins, one-tick latency is fine).
    std::unique_lock<std::mutex> lock(commandMutex_, std::try_to_lock);
    if(!lock.owns_lock() || !hasPendingCommand_) { return; }
    cmd = commandedData_;
    hasPendingCommand_ = false;
  }

  // Arm end-effector SE3 target (world). The wire quaternion is the SVA-frame
  // rotation, matching the reference explicit_compliance_controller round-trip.
  Eigen::Quaterniond q(cmd.eef_quat[0], cmd.eef_quat[1], cmd.eef_quat[2], cmd.eef_quat[3]);
  if(q.norm() < 1e-9) { q = Eigen::Quaterniond::Identity(); }
  q.normalize();
  sva::PTransformd eePose(q.toRotationMatrix(),
                          Eigen::Vector3d(cmd.eef_pos[0], cmd.eef_pos[1], cmd.eef_pos[2]));
  datastore().assign<sva::PTransformd>(EE_TARGET_KEY, eePose);

  datastore().assign<std::vector<double>>(ARM_POSTURE_KEY,
                                          std::vector<double>(cmd.posture_arm.begin(), cmd.posture_arm.end()));
  // posture_base drives the TriOrb PostureTask (joint-space base command); whether it
  // takes effect is governed by its weight (w_base_posture) in applyCommandsToTasks.
  datastore().assign<std::vector<double>>(BASE_POSTURE_KEY,
                                          std::vector<double>(cmd.posture_base.begin(), cmd.posture_base.end()));
  datastore().assign<Eigen::Vector3d>(
      BASE_VELOCITY_KEY, Eigen::Vector3d(cmd.velocity_base[0], cmd.velocity_base[1], cmd.velocity_base[2]));
  datastore().assign<double>(GRIPPER_OPENING_KEY, cmd.gripper_opening);

  // Per-task gains (mode + compliance), order [ee, posture_arm, base, base_posture].
  // Sentinel: a value < 0 keeps the current gain; >= 0 is adopted. Resolve against the
  // currently-stored gains so a partial command only touches what it sets.
  auto resolve = [&](const char * key, const std::array<double, 4> & incoming)
  {
    std::vector<double> cur = datastore().get<std::vector<double>>(key);
    for(int i = 0; i < 4; ++i)
    {
      const double v = incoming[static_cast<std::size_t>(i)];
      if(v >= 0.0) { cur[static_cast<std::size_t>(i)] = v; }
    }
    return cur;
  };
  const std::vector<double> w = resolve(TASK_WEIGHTS_KEY, cmd.task_weights);
  datastore().assign<std::vector<double>>(TASK_STIFFNESS_KEY, resolve(TASK_STIFFNESS_KEY, cmd.task_stiffness));
  datastore().assign<std::vector<double>>(TASK_DAMPING_KEY, resolve(TASK_DAMPING_KEY, cmd.task_damping_ratio));

  // Reject a degenerate weight set that leaves the arm (w_ee + w_arm) or the base
  // (w_base + w_base_posture) with no authority, so neither can float.
  if(w[0] + w[1] > 1e-9 && w[2] + w[3] > 1e-9)
  {
    datastore().assign<std::vector<double>>(TASK_WEIGHTS_KEY, w);
    weightsDegenerateWarned_ = false;
  }
  else if(!weightsDegenerateWarned_)
  {
    mc_rtc::log::warning("[CallmWbcController] ignoring task weights with no arm or base authority "
                         "(arm={}, base={}); keeping previous weights",
                         w[0] + w[1], w[2] + w[3]);
    weightsDegenerateWarned_ = true;
  }
}

void CallmWbcController::applyCommandsToTasks()
{
  // Per-task gains (mode + compliance), order [ee, posture_arm, base, base_posture].
  // Applied every tick so GUI/ROS changes take effect and the client can blend authority
  // (high w_ee = Cartesian, high w_posture_arm = joint-space) and shape compliance.
  const std::vector<double> w = datastore().get<std::vector<double>>(TASK_WEIGHTS_KEY);
  const std::vector<double> s = datastore().get<std::vector<double>>(TASK_STIFFNESS_KEY);
  const std::vector<double> zeta = datastore().get<std::vector<double>>(TASK_DAMPING_KEY);
  auto applyGains = [](const auto & task, double weight, double stiff, double damp)
  {
    if(!task) { return; }
    task->weight(weight);
    // `damp` is the damping ratio zeta when stiffness > 0 (D = 2*zeta*sqrt(K), zeta=1 =
    // critical); when stiffness == 0 it is the ABSOLUTE damping D, so a zero-stiffness
    // task becomes a pure velocity damper (e.g. immobilize the base) instead of going
    // inert (2*zeta*sqrt(0) = 0). See docs/gains.md.
    const double D = (stiff > 0.0) ? 2.0 * damp * std::sqrt(stiff) : damp;
    task->setGains(stiff, D);
  };
  applyGains(eeTask_, w[0], s[0], zeta[0]);
  applyGains(postureTask, w[1], s[1], zeta[1]);
  applyGains(baseTask_, w[2], s[2], zeta[2]);
  applyGains(triorbPostureTask_, w[3], s[3], zeta[3]);

  // Arm end-effector (active).
  if(eeTask_) { eeTask_->target(datastore().get<sva::PTransformd>(EE_TARGET_KEY)); }

  // Arm posture (active).
  if(postureTask)
  {
    const auto & arm = datastore().get<std::vector<double>>(ARM_POSTURE_KEY);
    std::map<std::string, std::vector<double>> target;
    for(size_t i = 0; i < armJointNames_.size() && i < arm.size(); ++i) { target[armJointNames_[i]] = {arm[i]}; }
    postureTask->target(target);
  }

  // Base posture (active): joint-space base target from posture_base. Whether it drives
  // the base is set by its weight (w_base_posture) vs the velocity-driven base task.
  if(triorbPostureTask_)
  {
    const auto & pb = datastore().get<std::vector<double>>(BASE_POSTURE_KEY);
    if(pb.size() >= baseJointNames_.size())
    {
      std::map<std::string, std::vector<double>> target;
      for(size_t i = 0; i < baseJointNames_.size(); ++i) { target[baseJointNames_[i]] = {pb[i]}; }
      triorbPostureTask_->target(target);
    }
  }

  // Base command: exactly one path drives the base transform task.
  if(baseTask_)
  {
    if(baseCommandMode_ == "velocity")
    {
      integrateBaseVelocity(); // active: velocity_base integrated into the base target
    }
    else
    {
      // inactive/overridable path: command the base by an absolute pose.
      baseTask_->target(datastore().get<sva::PTransformd>(BASE_TARGET_KEY));
      baseTask_->refVelB(sva::MotionVecd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    }
  }

  // Gripper (active): forward the opening to the mc_robotiq plugin (real gripper). This
  // is independent of whether the gripper MODEL is loaded -- only the plugin's datastore
  // call needs to exist.
  if(gripperCommandEnabled_ && datastore().has(gripperSetOpeningCall_))
  {
    double opening = datastore().get<double>(GRIPPER_OPENING_KEY);
    datastore().call(gripperSetOpeningCall_, opening);
  }
}

void CallmWbcController::integrateBaseVelocity()
{
  const auto vel = datastore().get<Eigen::Vector3d>(BASE_VELOCITY_KEY); // (vx, vy, wyaw) in the base body frame
  const double vx = vel.x(), vy = vel.y(), wz = vel.z();
  const double dt = solver().dt();

  // Re-base the target off the CURRENT base pose each tick (a one-step-ahead "carrot"),
  // NOT off a persistent accumulator. This keeps the position error bounded at ~v*dt, so
  // the target can never wind up when the base cannot track the command (conflicting
  // tasks / constraints). The anchor is the MEASURED base pose (realRobot, updated by the
  // VisualOdometryObserver this tick) when VO is alive, else the control-robot base (which
  // moves by QP integration) so pure sim keeps progressing -- see docs/observer.md.
  // Trade-off: the base is velocity-controlled (motion via the refVelB feed-forward);
  // it does NOT catch up on lag. See docs/base_velocity_target.md.
  const sva::PTransformd Xcur = voAlive() ? realRobot("triorb").bodyPosW("base")
                                          : robots().robot("triorb").bodyPosW("base");
  const auto & R = Xcur.rotation();
  double yaw = std::atan2(R(0, 1), R(0, 0)); // SVA RotZ convention (planar base)
  const double c = std::cos(yaw), s = std::sin(yaw);

  Eigen::Vector3d t = Xcur.translation();
  t.x() += (c * vx - s * vy) * dt; // body -> world
  t.y() += (s * vx + c * vy) * dt;
  yaw += wz * dt;
  baseTargetPose_ = sva::PTransformd(sva::RotZ(yaw), t);

  baseTask_->target(baseTargetPose_);
  // Feed-forward the commanded body velocity so the QP tracks it smoothly.
  baseTask_->refVelB(sva::MotionVecd(Eigen::Vector3d(0.0, 0.0, wz), Eigen::Vector3d(vx, vy, 0.0)));
}

bool CallmWbcController::voAlive() const
{
  // The VisualOdometryObserver registers this call in its update() once it has applied a
  // fresh (within-timeout) VO fix to realRobots(). Absent (pure sim / no VO) -> false.
  return datastore().has("VO::isAlive") && datastore().call<bool>("VO::isAlive");
}

void CallmWbcController::exportBaseVelocity()
{
  // Only active when the TriorbBasePlugin is loaded (it registers this key).
  if(!datastore().has(triorbCmdKey_)) { return; }

  auto & tri = robots().robot("triorb");
  const double xd = tri.mbc().alpha[tri.jointIndexByName("base_x")][0]; // world-frame ẋ (commanded)
  const double yd = tri.mbc().alpha[tri.jointIndexByName("base_y")][0]; // world-frame ẏ (commanded)
  const double wz = tri.mbc().alpha[tri.jointIndexByName("base_yaw")][0]; // θ̇ (commanded)
  // Rotate the commanded world velocity into the *measured* base body frame the plugin
  // expects: use realRobot's yaw (VO) when alive, else the control-robot yaw (sim).
  const double yaw = voAlive()
                         ? realRobot("triorb").mbc().q[realRobot("triorb").jointIndexByName("base_yaw")][0]
                         : tri.mbc().q[tri.jointIndexByName("base_yaw")][0];
  const double c = std::cos(yaw), s = std::sin(yaw);
  // world -> body; the plugin is configured with command_in_world_frame: false.
  const Eigen::Vector3d body(c * xd + s * yd, -s * xd + c * yd, wz);
  datastore().assign<Eigen::Vector3d>(triorbCmdKey_, body);
}

void CallmWbcController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  // 1. Initial poses, set BEFORE the coupling contacts are added. The TriOrb root
  // (`world`) stays at the origin; the base is carried by its base_x/base_y/base_yaw
  // joints. The arm's floating base is planted directly ON the TriOrb `mount` frame,
  // so the UR5e "Base" surface coincides with the runtime-added "ArmMount" surface and
  // the URDF (base_to_mount in triorb.urdf: currently Rz(-90deg), z=0.60) is the single
  // source of truth for the mounting orientation/height. This also composes correctly
  // with any nonzero base reset pose (unlike a hard-coded world transform).
  robots().robot("triorb").posW(sva::PTransformd::Identity());
  robots().robot(0).posW(robots().robot("triorb").bodyPosW("mount"));

  // 2. Rigid arm<->base attachment (all 6 dof constrained). Added AFTER posW so the
  // contact frame captures the intended relative pose.
  addContact({"triorb", "ur5e", armMountSurface_, "Base"});

  // 3. WBC tasks.
  eeTask_ = std::make_shared<mc_tasks::SurfaceTransformTask>("Tool", robots(), 0, eeStiffness_, eeWeight_);
  eeTask_->reset(); // target := current Tool pose
  solver().addTask(eeTask_);

  baseTask_ = std::make_shared<mc_tasks::TransformTask>("base", robots(), 1, baseStiffness_, baseWeight_);
  baseTask_->reset(); // target := current base pose
  solver().addTask(baseTask_); // gains are (re)applied every tick from the datastore, see applyCommandsToTasks
  baseTargetPose_ = baseTask_->target();

  // Per-robot constraint + light posture for the base (regularizes base_x/base_y/base_yaw).
  triorbKinematics_ = std::make_unique<mc_solver::KinematicsConstraint>(robots(), 1, solver().dt());
  solver().addConstraintSet(triorbKinematics_);
  triorbPostureTask_ =
      std::make_shared<mc_tasks::PostureTask>(solver(), 1, triorbPostureStiffness_, triorbPostureWeight_);
  solver().addTask(triorbPostureTask_);

  // 4. Robotiq gripper: bolt it onto the UR5e tool and regularize its knuckle joints.
  if(gripperEnabled_ && robots().hasRobot(gripperRobot_))
  {
    const unsigned int gi = robots().robot(gripperRobot_).robotIndex();
    // Place the gripper base at the tool with the module's default mounting transform
    // (RobotiqGripperRobotModule::defaultMountingTransform() == RotZ(pi)), then freeze
    // the attachment with a rigid Tool<->Base contact.
    const sva::PTransformd toolPose = robots().robot("ur5e").surfacePose("Tool");
    const sva::PTransformd mount(sva::RotZ(M_PI));
    robots().robot(gripperRobot_).posW(mount * toolPose);
    addContact({"ur5e", gripperRobot_, "Tool", gripperBaseSurface_});

    gripperKinematics_ = std::make_unique<mc_solver::KinematicsConstraint>(robots(), gi, solver().dt());
    solver().addConstraintSet(gripperKinematics_);
    gripperPostureTask_ =
        std::make_shared<mc_tasks::PostureTask>(solver(), gi, gripperPostureStiffness_, gripperPostureWeight_);
    solver().addTask(gripperPostureTask_);
  }

  // 5. UR5e standby posture (resolves arm redundancy in the null space of the EE task).
  postureTask->stiffness(ur5ePostureStiffness_);
  postureTask->weight(ur5ePostureWeight_);
  const std::vector<double> armStandby = {0.0, -M_PI / 2, M_PI / 2, 0.0, 0.0, 0.0};

  // 6. Seed the commanded targets so the robot holds still until a commander moves them.
  datastore().assign<sva::PTransformd>(EE_TARGET_KEY, eeTask_->target());
  datastore().assign<sva::PTransformd>(BASE_TARGET_KEY, baseTask_->target());
  datastore().assign<Eigen::Vector3d>(BASE_VELOCITY_KEY, Eigen::Vector3d::Zero());
  datastore().assign<std::vector<double>>(ARM_POSTURE_KEY, armStandby);
  datastore().assign<std::vector<double>>(BASE_POSTURE_KEY, std::vector<double>(baseJointNames_.size(), 0.0));
  datastore().assign<double>(GRIPPER_OPENING_KEY, 0.0);
  // Each episode starts from the YAML-default gains; the client picks its mode/compliance
  // via the WbcData task_weights / task_stiffness / task_damping_ratio fields. Damping is
  // seeded critical (zeta = 1) for every task; shape it at runtime via task_damping_ratio.
  datastore().assign<std::vector<double>>(
      TASK_WEIGHTS_KEY, std::vector<double>{eeWeight_, ur5ePostureWeight_, baseWeight_, triorbPostureWeight_});
  datastore().assign<std::vector<double>>(
      TASK_STIFFNESS_KEY,
      std::vector<double>{eeStiffness_, ur5ePostureStiffness_, baseStiffness_, triorbPostureStiffness_});
  datastore().assign<std::vector<double>>(TASK_DAMPING_KEY, std::vector<double>{1.0, 1.0, 1.0, 1.0});
  weightsDegenerateWarned_ = false;

  // Push the seeds onto the freshly created tasks once, so the first run() is consistent.
  applyCommandsToTasks();

  mc_rtc::log::success("CallmWbcController reset done");
}

// ---------------------------------------------------------------------------
// ROS2 interface (own context + single-threaded executor on a spin thread).
// Mirrors explicit_compliance_controller so commands never touch mc_rtc objects
// off the control loop: the callback only fills a mutex-guarded WbcData buffer.
// ---------------------------------------------------------------------------
void CallmWbcController::setupRos()
{
  rosContext_ = std::make_shared<rclcpp::Context>();
  // mc_rtc's ROS plugin already owns the global rclcpp logging system. Initializing it
  // again from our private context triggers "logging was initialized more than once" and
  // a double logging-shutdown at teardown (segfault). Defer logging to mc_rtc's context.
  rclcpp::InitOptions initOptions;
  initOptions.auto_initialize_logging(false);
  rosContext_->init(0, nullptr, initOptions);

  rclcpp::NodeOptions options;
  options.context(rosContext_);
  rosNode_ = std::make_shared<rclcpp::Node>(rosNodeName_, options);
  rosCallbackGroup_ = rosNode_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = rosCallbackGroup_;
  commandSubscriber_ = rosNode_->create_subscription<std_msgs::msg::Float64MultiArray>(
      commandTopic_, rclcpp::QoS(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { handleCommand(msg); }, sub_options);

  // Base localization is no longer subscribed here: the VisualOdometryObserver owns the
  // VO topic and grounds realRobots() (see docs/observer.md).
  if(publishMeasured_)
  {
    measuredPublisher_ = rosNode_->create_publisher<std_msgs::msg::Float64MultiArray>(measuredTopic_, rclcpp::QoS(1));
  }

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = rosContext_;
  rosExecutor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
  rosExecutor_->add_node(rosNode_);
  rosSpinThread_ = std::thread([this]() { rosExecutor_->spin(); });

  mc_rtc::log::info("[CallmWbcController] ROS node '{}' listening on '{}'", rosNodeName_, commandTopic_);
}

void CallmWbcController::stopRos()
{
  if(rosExecutor_) { rosExecutor_->cancel(); }
  if(rosSpinThread_.joinable()) { rosSpinThread_.join(); }
  if(rosNode_ && rosExecutor_) { rosExecutor_->remove_node(rosNode_); }
  commandSubscriber_.reset();
  measuredPublisher_.reset();
  rosNode_.reset();
  rosExecutor_.reset();
  rosCallbackGroup_.reset();
  if(rosContext_)
  {
    rosContext_->shutdown("CallmWbcController shutdown");
    rosContext_.reset();
  }
}

void CallmWbcController::handleCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  WbcData unpacked;
  if(!WbcData::unpack(msg->data, unpacked))
  {
    mc_rtc::log::warning("[CallmWbcController] {} size mismatch: expected {}, got {}", commandTopic_, WbcData::SIZE,
                         msg->data.size());
    return;
  }
  std::lock_guard<std::mutex> lock(commandMutex_);
  commandedData_ = unpacked;
  hasPendingCommand_ = true;
}

WbcData CallmWbcController::collectMeasured() const
{
  WbcData m;
  const auto & ur5e = robots().robot("ur5e");
  const sva::PTransformd toolPose = ur5e.surfacePose("Tool");
  const Eigen::Quaterniond quat(toolPose.rotation());
  m.eef_pos = {toolPose.translation().x(), toolPose.translation().y(), toolPose.translation().z()};
  m.eef_quat = {quat.w(), quat.x(), quat.y(), quat.z()};
  for(size_t i = 0; i < armJointNames_.size(); ++i)
  {
    m.posture_arm[i] = ur5e.mbc().q[ur5e.jointIndexByName(armJointNames_[i])][0];
  }

  const auto & tri = robots().robot("triorb");
  for(size_t i = 0; i < baseJointNames_.size(); ++i)
  {
    m.posture_base[i] = tri.mbc().q[tri.jointIndexByName(baseJointNames_[i])][0];
  }
  // Base body velocity from the integrated base joint velocities (world -> body).
  const double xd = tri.mbc().alpha[tri.jointIndexByName("base_x")][0];
  const double yd = tri.mbc().alpha[tri.jointIndexByName("base_y")][0];
  const double thd = tri.mbc().alpha[tri.jointIndexByName("base_yaw")][0];
  const double yaw = m.posture_base[2];
  const double c = std::cos(yaw), s = std::sin(yaw);
  m.velocity_base = {c * xd + s * yd, -s * xd + c * yd, thd};

  if(datastore().has("RobotiqGripper::opening"))
  {
    m.gripper_opening = datastore().call<double>("RobotiqGripper::opening");
  }

  // Echo the currently-active per-task gains so the client can read back its mode.
  const auto & wVec = datastore().get<std::vector<double>>(TASK_WEIGHTS_KEY);
  const auto & sVec = datastore().get<std::vector<double>>(TASK_STIFFNESS_KEY);
  const auto & zVec = datastore().get<std::vector<double>>(TASK_DAMPING_KEY);
  for(std::size_t i = 0; i < m.task_weights.size(); ++i)
  {
    if(i < wVec.size()) { m.task_weights[i] = wVec[i]; }
    if(i < sVec.size()) { m.task_stiffness[i] = sVec[i]; }
    if(i < zVec.size()) { m.task_damping_ratio[i] = zVec[i]; }
  }
  return m;
}

void CallmWbcController::publishMeasured()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = collectMeasured().pack();
  measuredPublisher_->publish(msg);
}

CONTROLLER_CONSTRUCTOR("CallmWbcController", CallmWbcController)
