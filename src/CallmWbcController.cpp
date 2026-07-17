#include "CallmWbcController.h"

#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>

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
#include <memory>
#include <utility>

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
  if(s == "none") { return mc_solver::FeedbackType::None; }
  if(s == "joints") { return mc_solver::FeedbackType::Joints; }
  if(s == "joints_velocity") { return mc_solver::FeedbackType::JointsWVelocity; }
  if(s == "observed") { return mc_solver::FeedbackType::ObservedRobots; }
  if(s == "observed_real") { return mc_solver::FeedbackType::ClosedLoopIntegrateReal; }
  mc_rtc::log::error_and_throw("[CallmWbcController] unknown feedback '{}'; expected one of: none, joints, "
                               "joints_velocity, observed, observed_real",
                               s);
}

// Wire<->pose rotation boundary. Wire quaternions are standard ROS/Hamilton (active,
// tf2/RViz); sva::PTransformd stores the transposed frame rotation. Transpose here only.

/** World PTransformd from a wire Hamilton quaternion (w,x,y,z) + position. */
sva::PTransformd poseFromWire(const std::array<double, 4> & wxyz, const std::array<double, 3> & xyz)
{
  Eigen::Quaterniond q(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
  if(q.norm() < 1e-9) { q = Eigen::Quaterniond::Identity(); }
  q.normalize();
  const Eigen::Matrix3d E = q.toRotationMatrix().transpose();
  return sva::PTransformd(E, Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
}

/** Wire Hamilton quaternion (w,x,y,z) of a world PTransformd's orientation. */
std::array<double, 4> wireQuat(const sva::PTransformd & X)
{
  const Eigen::Quaterniond q(X.rotation().transpose());
  return {q.w(), q.x(), q.y(), q.z()};
}
} // namespace

std::vector<mc_rbdyn::RobotModulePtr> CallmWbcController::robotModules(mc_rbdyn::RobotModulePtr rm,
                                                                       const mc_rtc::Configuration & config)
{
  // The UR5e arm, the TriOrb base and (optionally) the Robotiq gripper are fused into ONE
  // robot module with mc_rbdyn::RobotModule::connect, so the QP sees a single kinematic
  // tree instead of separate robots held together by contacts. See
  // docs/connect_migration.md.
  //
  // This runs from the member-initializer list (see the constructor), i.e. BEFORE
  // MCController loads any robot -- which is exactly the situation connect() is meant
  // for. mc_rtc does NOT support connecting robots that are already loaded in a
  // controller (doc/_i18n/en/tutorials/advanced/new-robot.html, "Connecting existing
  // robot modules"), so the attachment must be decided here and not in reset().
  //
  // robot 0: callm (triorb + UR5e [+ gripper]) ; robot 1: env/ground
  std::string mergedName = "callm";
  if(config.has("connect")) { config("connect")("name", mergedName); }

  auto triorb = mc_rbdyn::RobotLoader::get_robot_module("triorb");
  if(!triorb) { mc_rtc::log::error_and_throw("[CallmWbcController] Could not load the 'triorb' robot module"); }
  if(!rm) { mc_rtc::log::error_and_throw("[CallmWbcController] No main robot module provided"); }

  // `triorb` is `this` so the merged root is its `odom` link and base_x/base_y/base_yaw
  // keep their names (connect only prefixes `other`'s entities) -- the VisualOdometry
  // observer's planar_joints depend on that.
  //
  // `mount` (triorb.urdf) already carries the mounting height/orientation (z = 0.60,
  // Rz(-90deg)), and the UR5e root link `base_link` is planted straight onto it, so both
  // connection transforms are identity. This reproduces what the old Base<->Base contact
  // did (posW(bodyPosW("mount")) + two coincident planar surfaces), but structurally.
  //
  // The default connection joint is Fixed (dof 0), so it is NOT appended to the merged
  // ref_joint_order: rjo = [base_x, base_y, base_yaw] + [6 UR5e joints].
  //
  // The prefix is empty because we want the arm's joints and `tool0`/`Tool` to keep their
  // names (armJointNames_, the EE task and the gripper mount all depend on that), and the
  // two modules' joints are in fact disjoint. Their BODIES are not: ur_description defines a link literally called
  // `base` -- the ROS-Industrial base frame, a -pi rotation of `base_link` (joint
  // `base_link-base_fixed_joint`) -- and the TriOrb's box link is also called `base`.
  // connect() throws "Body name: base already exists" on that clash, so the arm's one is
  // remapped. It is unused here: we mount on `base_link`, and the `base` convex that
  // arm<->base collision avoidance targets is the TriOrb's box.
  //
  // `other`'s root joint is dropped by connect(), so passing the UR5eFloatingBase variant
  // is fine -- its floating base is eliminated.
  auto merged = triorb->connect(*rm, "mount", "base_link", "",
                                mc_rbdyn::RobotModule::ConnectionParameters{}.name(mergedName).bodyMapping(
                                    {{"base", "ur5e_base"}}));

  // 2. Gripper onto the tool. `gripper.simulate` controls the MODEL only; commanding the
  // real gripper goes through the mc_robotiq plugin (gripper.command) and is independent
  // of it. Asking for the model and not getting it is an ERROR, not a warning: a silent
  // fallback here is invisible in a 200-line startup log and leaves you wondering why the
  // gripper never appeared.
  bool gripperSimulate = true;
  std::string gripperModule = "Robotiq2f85Gripper";
  if(config.has("gripper"))
  {
    auto g = config("gripper");
    g("simulate", gripperSimulate);
    g("model", gripperModule);
  }
  if(gripperSimulate)
  {
    auto gm = mc_rbdyn::RobotLoader::get_robot_module(gripperModule);
    if(!gm)
    {
      mc_rtc::log::error_and_throw("[CallmWbcController] gripper.simulate is on but the gripper module '{}' could not "
                                   "be loaded. Check that mc_robot_tools is installed and on the module path, or set "
                                   "gripper.simulate: false",
                                   gripperModule);
    }
    // Same shape as mc_kinova's attachTool (mc_kinova/src/module.cpp:112-114):
    // parent.connect(tool, parent_frame, tool.baseFrame(), prefix, X_other_connection(
    // tool.defaultMountingTransform())). We inline the two constants instead of linking
    // mc_robot_tools for ConnectableRobotModule, because this package keeps robot modules
    // as runtime plugins rather than link-time dependencies (see src/CMakeLists.txt). Both
    // values are verified against mc_robot_tools/robotiq_gripper/src/robotiq_gripper.cpp:
    // baseFrame() is "<prefix>_base_link" (:21-24) and defaultMountingTransform() is
    // RotZ(pi) (:37-40). RotZ(pi) is its own inverse, so this reproduces the old
    // posW(RotZ(pi) * toolPose).
    //
    // Unlike the arm, this connect takes a NON-EMPTY prefix. The gripper's links come from
    // robotiq_description's xacro macro, which is resolved at build time and is not
    // inspectable from this repo, so we cannot enumerate its body/joint names to prove
    // they do not clash (the arm taught us that lesson: ur_description's `base`). A prefix
    // makes a clash impossible whatever they turn out to be, and it also renames the
    // gripper's rsdf surface -- which IS named "Base", the same as the UR5e's, and which
    // connect() would silently clobber since it validates convex/sensor/gripper/device
    // name collisions but NOT surface ones.
    //
    // Nothing downstream hardcodes the gripper's names: the posture task's joints are
    // derived from the merged model in the constructor, and the command path goes through
    // the mc_robotiq plugin, not the model.
    //
    // `other_body` is given UNPREFIXED here -- connect() applies the prefix itself when it
    // looks the body up (RobotModule_connect.cpp:168).
    const bool is140 = gripperModule.find("140") != std::string::npos;
    const std::string gBaseLink = is140 ? "robotiq_140_base_link" : "robotiq_85_base_link";
    merged = merged.connect(*gm, "tool0", gBaseLink, "gripper_",
                            mc_rbdyn::RobotModule::ConnectionParameters{}.name(mergedName).X_other_connection(
                                sva::PTransformd(sva::RotZ(M_PI))));
  }

  std::vector<mc_rbdyn::RobotModulePtr> modules;
  modules.push_back(std::make_shared<mc_rbdyn::RobotModule>(std::move(merged)));
  modules.push_back(mc_rbdyn::RobotLoader::get_robot_module("env/ground"));
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
  loadGains("callm_ee_task", callmEeStiffness_, callmEeWeight_);
  loadGains("triorb_base_task", triorbBaseStiffness_, triorbBaseWeight_);
  loadGains("ur5e_posture", ur5ePostureStiffness_, ur5ePostureWeight_);
  loadGains("triorb_posture", triorbPostureStiffness_, triorbPostureWeight_);
  loadGains("gripper_posture", gripperPostureStiffness_, gripperPostureWeight_);

  // ---- Base command routing (velocity is the active path) ------------------
  if(config.has("base_command")) { config("base_command")("mode", baseCommandMode_); }

  // ---- Gripper command switch ----------------------------------------------
  // Independent of the MODEL: the mc_robotiq plugin talks to the physical gripper over a
  // socket and moves no model joints, so the real gripper can be driven with
  // gripper.simulate: false.
  if(config.has("gripper"))
  {
    auto g = config("gripper");
    g("set_opening_call", gripperSetOpeningCall_);
    g("command", gripperCommandEnabled_);
  }

  // The gripper's joints are the merged robot's actuated joints that are neither arm nor
  // base. Deriving them from the model rather than hardcoding keeps this independent of
  // both robotiq_description's naming and the prefix robotModules() chose, and covers the
  // mimic/knuckle joints without enumerating them.
  for(const auto & j : robot().mb().joints())
  {
    if(j.dof() == 0) { continue; } // fixed joints, including the connect joints
    const auto & n = j.name();
    const bool isArm = std::find(armJointNames_.begin(), armJointNames_.end(), n) != armJointNames_.end();
    const bool isBase = std::find(baseJointNames_.begin(), baseJointNames_.end(), n) != baseJointNames_.end();
    if(!isArm && !isBase) { gripperJointNames_.push_back(n); }
  }
  gripperEnabled_ = !gripperJointNames_.empty(); // model connected in (independent of command)

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

  // ---- Constraints + arm posture -------------------------------------------
  // robot 0 is the merged robot, so mc_rtc's built-in robot-0 constraints cover the base,
  // the arm and the gripper in one go: no per-part KinematicsConstraint is needed any
  // more, and selfCollisionConstraint is seeded from the merged module's
  // minimalSelfCollisions (connect() carries over the UR5e's -- see
  // docs/connect_migration.md).
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint); // merged robot joint limits (base + arm + gripper)
  solver().addConstraintSet(selfCollisionConstraint); // merged robot self-collisions
  // The built-in posture task spans the whole merged robot, so restrict it to the arm
  // joints: it is the `ur5e_posture` entry of the WbcData 4-task contract. The base and
  // the gripper get their own posture tasks over disjoint joint sets in reset(). Renamed
  // for the same reason as those -- PostureTask names itself after the robot, so all three
  // would be "posture_callm". The name must be set before addTask().
  postureTask->name("ur5e_posture");
  postureTask->selectActiveJoints(solver(), armJointNames_);
  solver().addTask(postureTask);
  // The arm<->base and tool<->gripper attachments are structural now (connect()), so the
  // QP has no contacts at all: the base is carried by its base_x/base_y/base_yaw joints
  // and the ground robot is visual only.
  solver().setContacts({});

  // ---- Arm <-> base collision avoidance ------------------------------------
  // mc_rtc auto-builds an sch::S_Box collision convex named "base" for the TriOrb
  // base link directly from the URDF <box> collision primitive, so no extra hull
  // file is needed. We guard the distal arm links against that box. base_link and
  // shoulder_link are intentionally excluded: the arm base is rigidly mounted on
  // top of the base, so they sit permanently against it by design.
  //
  // Both sides live in the merged robot now, so these pairs ARE self-collisions: they go
  // into the existing selfCollisionConstraint rather than through addCollisions(). Going
  // through addCollisions(name, name, ...) would build a SECOND CollisionsConstraint over
  // pair (0,0); both derive their GUI category from the robot names, so the second one
  // fails to register its "Automatic monitor" checkbox in Collisions/callm/callm. This is
  // also how mc_rtc itself seeds the module's own self-collisions.
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
    selfCollisionConstraint->addCollisions(solver(), armBaseCollisions);
  }

  setupTargetsIO();
  setupRos();

  mc_rtc::log::success("CallmWbcController init done (merged robot: {}, {} dof; gripper model: {}, command: {})",
                       robot().name(), robot().mb().nrDof(),
                       gripperEnabled_ ? fmt::format("{} joints", gripperJointNames_.size()) : "not simulated",
                       gripperCommandEnabled_ ? "on" : "off");
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
  datastore().make<sva::PTransformd>(CALLM_EE_TARGET_KEY, sva::PTransformd::Identity());
  datastore().make<sva::PTransformd>(TRIORB_BASE_TARGET_KEY, sva::PTransformd(Eigen::Vector3d(0.0, 0.0, baseHeight_)));
  datastore().make<Eigen::Vector3d>(TRIORB_BASE_VELOCITY_KEY, Eigen::Vector3d::Zero());
  datastore().make<std::vector<double>>(UR5E_POSTURE_KEY, std::vector<double>(armJointNames_.size(), 0.0));
  datastore().make<std::vector<double>>(TRIORB_POSTURE_KEY, std::vector<double>(baseJointNames_.size(), 0.0));
  datastore().make<double>(GRIPPER_OPENING_KEY, 0.0);
  // Per-task gains, order [callm_ee, ur5e_posture, triorb_base, triorb_posture], seeded from the YAML
  // defaults (damping ratio = 1 => critically damped). A client picks a "mode" via the
  // weights and shapes compliance via stiffness/damping. See applyPendingCommand for
  // the <0 = keep-default sentinel.
  datastore().make<std::vector<double>>(
      TASK_WEIGHTS_KEY, std::vector<double>{callmEeWeight_, ur5ePostureWeight_, triorbBaseWeight_, triorbPostureWeight_});
  datastore().make<std::vector<double>>(
      TASK_STIFFNESS_KEY,
      std::vector<double>{callmEeStiffness_, ur5ePostureStiffness_, triorbBaseStiffness_, triorbPostureStiffness_});
  datastore().make<std::vector<double>>(TASK_DAMPING_KEY, std::vector<double>{1.0, 1.0, 1.0, 1.0});

  gui()->addElement(
      {"CallmWbc"},
      mc_rtc::gui::Transform(
          "callm_ee target [world]", [this]() -> sva::PTransformd
          { return datastore().get<sva::PTransformd>(CALLM_EE_TARGET_KEY); },
          [this](const sva::PTransformd & p) { datastore().assign<sva::PTransformd>(CALLM_EE_TARGET_KEY, p); }),
      mc_rtc::gui::ArrayInput(
          "triorb_base velocity [vx, vy, wyaw] (body)", {"vx", "vy", "wyaw"},
          [this]() -> Eigen::Vector3d { return datastore().get<Eigen::Vector3d>(TRIORB_BASE_VELOCITY_KEY); },
          [this](const Eigen::Vector3d & v) { datastore().assign<Eigen::Vector3d>(TRIORB_BASE_VELOCITY_KEY, v); }),
      mc_rtc::gui::ArrayInput(
          "ur5e_posture [rad]", armJointNames_,
          [this]() -> std::vector<double> { return datastore().get<std::vector<double>>(UR5E_POSTURE_KEY); },
          [this](const std::vector<double> & q) { datastore().assign<std::vector<double>>(UR5E_POSTURE_KEY, q); }),
      mc_rtc::gui::NumberSlider(
          "Gripper opening (0=open, 1=closed)", [this]() { return datastore().get<double>(GRIPPER_OPENING_KEY); },
          [this](double v) { datastore().assign<double>(GRIPPER_OPENING_KEY, std::max(0.0, std::min(1.0, v))); }, 0.0,
          1.0),
      mc_rtc::gui::ArrayInput(
          "Task weights", {"callm_ee", "ur5e_posture", "triorb_base", "triorb_posture"},
          [this]() -> std::vector<double> { return datastore().get<std::vector<double>>(TASK_WEIGHTS_KEY); },
          [this](const std::vector<double> & v)
          { datastore().assign<std::vector<double>>(TASK_WEIGHTS_KEY, clampNonNeg(v)); }),
      mc_rtc::gui::ArrayInput(
          "Task stiffness", {"callm_ee", "ur5e_posture", "triorb_base", "triorb_posture"},
          [this]() -> std::vector<double> { return datastore().get<std::vector<double>>(TASK_STIFFNESS_KEY); },
          [this](const std::vector<double> & v)
          { datastore().assign<std::vector<double>>(TASK_STIFFNESS_KEY, clampNonNeg(v)); }),
      mc_rtc::gui::ArrayInput(
          "Task damping ratio", {"callm_ee", "ur5e_posture", "triorb_base", "triorb_posture"},
          [this]() -> std::vector<double> { return datastore().get<std::vector<double>>(TASK_DAMPING_KEY); },
          [this](const std::vector<double> & v)
          { datastore().assign<std::vector<double>>(TASK_DAMPING_KEY, clampNonNeg(v)); }));

  // Inactive/overridable base-pose command path (used only when base_command.mode = "pose").
  gui()->addElement(
      {"CallmWbc", "triorb_base pose (inactive path)"},
      mc_rtc::gui::Transform(
          "triorb_base target [world]", [this]() -> sva::PTransformd
          { return datastore().get<sva::PTransformd>(TRIORB_BASE_TARGET_KEY); },
          [this](const sva::PTransformd & p) { datastore().assign<sva::PTransformd>(TRIORB_BASE_TARGET_KEY, p); }),
      mc_rtc::gui::Button("Sync targets to current robot",
                          [this]()
                          {
                            datastore().assign<sva::PTransformd>(CALLM_EE_TARGET_KEY, robot().surfacePose("Tool"));
                            datastore().assign<sva::PTransformd>(TRIORB_BASE_TARGET_KEY, robot().bodyPosW("base"));
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

  // Arm EE target: Tool pose in world (Hamilton quat, transposed in poseFromWire).
  datastore().assign<sva::PTransformd>(CALLM_EE_TARGET_KEY, poseFromWire(cmd.eef_quat, cmd.eef_pos));

  datastore().assign<std::vector<double>>(UR5E_POSTURE_KEY,
                                          std::vector<double>(cmd.posture_arm.begin(), cmd.posture_arm.end()));
  // posture_base drives the TriOrb PostureTask (joint-space base command); whether it
  // takes effect is governed by its weight (triorb_posture) in applyCommandsToTasks.
  datastore().assign<std::vector<double>>(TRIORB_POSTURE_KEY,
                                          std::vector<double>(cmd.posture_base.begin(), cmd.posture_base.end()));
  datastore().assign<Eigen::Vector3d>(
      TRIORB_BASE_VELOCITY_KEY, Eigen::Vector3d(cmd.velocity_base[0], cmd.velocity_base[1], cmd.velocity_base[2]));
  datastore().assign<double>(GRIPPER_OPENING_KEY, cmd.gripper_opening);

  // Per-task gains (mode + compliance), order [callm_ee, ur5e_posture, triorb_base, triorb_posture].
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

  // Reject a degenerate weight set that leaves the arm (callm_ee + ur5e_posture) or the
  // base (triorb_base + triorb_posture) with no authority, so neither can float.
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
  // Per-task gains (mode + compliance), order [callm_ee, ur5e_posture, triorb_base, triorb_posture].
  // Applied every tick so GUI/ROS changes take effect and the client can blend authority
  // (high callm_ee = Cartesian, high ur5e_posture = joint-space) and shape compliance.
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
  applyGains(callmEeTask_, w[0], s[0], zeta[0]);
  applyGains(postureTask, w[1], s[1], zeta[1]);
  applyGains(triorbBaseTask_, w[2], s[2], zeta[2]);
  applyGains(triorbPostureTask_, w[3], s[3], zeta[3]);

  // Arm end-effector (active).
  if(callmEeTask_) { callmEeTask_->target(datastore().get<sva::PTransformd>(CALLM_EE_TARGET_KEY)); }

  // Arm posture (active).
  if(postureTask)
  {
    const auto & arm = datastore().get<std::vector<double>>(UR5E_POSTURE_KEY);
    std::map<std::string, std::vector<double>> target;
    for(size_t i = 0; i < armJointNames_.size() && i < arm.size(); ++i) { target[armJointNames_[i]] = {arm[i]}; }
    postureTask->target(target);
  }

  // Base posture (active): joint-space base target from posture_base. Whether it drives
  // the base is set by its weight (triorb_posture) vs the velocity-driven triorb_base task.
  if(triorbPostureTask_)
  {
    const auto & pb = datastore().get<std::vector<double>>(TRIORB_POSTURE_KEY);
    if(pb.size() >= baseJointNames_.size())
    {
      std::map<std::string, std::vector<double>> target;
      for(size_t i = 0; i < baseJointNames_.size(); ++i) { target[baseJointNames_[i]] = {pb[i]}; }
      triorbPostureTask_->target(target);
    }
  }

  // Base command: exactly one path drives the base transform task.
  if(triorbBaseTask_)
  {
    if(baseCommandMode_ == "velocity")
    {
      integrateBaseVelocity(); // active: velocity_base integrated into the base target
    }
    else
    {
      // inactive/overridable path: command the base by an absolute pose.
      triorbBaseTask_->target(datastore().get<sva::PTransformd>(TRIORB_BASE_TARGET_KEY));
      triorbBaseTask_->refVelB(sva::MotionVecd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    }
  }

  // Gripper (active): forward the opening to the mc_robotiq plugin, which drives the REAL
  // gripper over a socket. The gripper is not modelled here at all -- only the plugin's
  // datastore call needs to exist.
  if(gripperCommandEnabled_ && datastore().has(gripperSetOpeningCall_))
  {
    double opening = datastore().get<double>(GRIPPER_OPENING_KEY);
    datastore().call(gripperSetOpeningCall_, opening);
  }
}

void CallmWbcController::integrateBaseVelocity()
{
  const auto vel = datastore().get<Eigen::Vector3d>(TRIORB_BASE_VELOCITY_KEY); // (vx, vy, wyaw) in the base body frame
  const double vx = vel.x(), vy = vel.y(), wz = vel.z();
  const double dt = solver().dt();

  // Re-base the target off the CURRENT base pose each tick (a one-step-ahead "carrot"),
  // NOT off a persistent accumulator. This keeps the position error bounded at ~v*dt, so
  // the target can never wind up when the base cannot track the command (conflicting
  // tasks / constraints). The anchor is the MEASURED base pose (realRobot) only under a
  // closed-loop feedback mode with fresh VO -- the state the QP builds at; otherwise the
  // control-robot base (self-rebase), so open-loop VO does NOT move the control base and
  // pure sim keeps progressing. See docs/observer.md.
  // Trade-off: the base is velocity-controlled (motion via the refVelB feed-forward);
  // it does NOT catch up on lag. See docs/base_velocity_target.md.
  const sva::PTransformd Xcur = useMeasuredBase() ? realRobot().bodyPosW("base") : robot().bodyPosW("base");
  const auto & R = Xcur.rotation();
  double yaw = std::atan2(R(0, 1), R(0, 0)); // SVA RotZ convention (planar base)
  const double c = std::cos(yaw), s = std::sin(yaw);

  Eigen::Vector3d t = Xcur.translation();
  t.x() += (c * vx - s * vy) * dt; // body -> world
  t.y() += (s * vx + c * vy) * dt;
  yaw += wz * dt;
  triorbBaseTargetPose_ = sva::PTransformd(sva::RotZ(yaw), t);

  triorbBaseTask_->target(triorbBaseTargetPose_);
  // Feed-forward the commanded body velocity so the QP tracks it smoothly.
  triorbBaseTask_->refVelB(sva::MotionVecd(Eigen::Vector3d(0.0, 0.0, wz), Eigen::Vector3d(vx, vy, 0.0)));
}

bool CallmWbcController::voAlive() const
{
  // The VisualOdometryObserver registers this call in its update() once it has applied a
  // fresh (within-timeout) VO fix to realRobots(). Absent (pure sim / no VO) -> false.
  return datastore().has("VO::isAlive") && datastore().call<bool>("VO::isAlive");
}

bool CallmWbcController::useMeasuredBase() const
{
  // Reference the measured base (realRobots) only when the QP is actually closing the loop
  // on realRobots (observed / observed_real) AND VO is fresh. Under open-loop (none/joints)
  // the control base is never grounded to realRobots -- the base re-bases off the control
  // robot itself, so VO updates realRobots() only and does NOT move the control base.
  const bool closedLoop = feedbackType_ == mc_solver::FeedbackType::ObservedRobots
                          || feedbackType_ == mc_solver::FeedbackType::ClosedLoopIntegrateReal;
  return closedLoop && voAlive();
}

void CallmWbcController::exportBaseVelocity()
{
  // Only active when the TriorbBasePlugin is loaded (it registers this key).
  if(!datastore().has(triorbCmdKey_)) { return; }

  auto & tri = robot();
  const double xd = tri.mbc().alpha[tri.jointIndexByName("base_x")][0]; // world-frame ẋ (commanded)
  const double yd = tri.mbc().alpha[tri.jointIndexByName("base_y")][0]; // world-frame ẏ (commanded)
  const double wz = tri.mbc().alpha[tri.jointIndexByName("base_yaw")][0]; // θ̇ (commanded)
  // Rotate the commanded world velocity into the *measured* base body frame the plugin
  // expects: use realRobot's yaw (VO) under closed-loop feedback, else the control yaw.
  const double yaw = useMeasuredBase() ? realRobot().mbc().q[realRobot().jointIndexByName("base_yaw")][0]
                                       : tri.mbc().q[tri.jointIndexByName("base_yaw")][0];
  const double c = std::cos(yaw), s = std::sin(yaw);
  // world -> body; the plugin is configured with command_in_world_frame: false.
  const Eigen::Vector3d body(c * xd + s * yd, -s * xd + c * yd, wz);
  datastore().assign<Eigen::Vector3d>(triorbCmdKey_, body);
}

void CallmWbcController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  // 1. Initial pose. There is a single robot now, so this is one call: the merged root
  // (triorb's `odom` link) goes to the origin and the base is carried by its
  // base_x/base_y/base_yaw joints. The arm follows through the merged kinematic tree --
  // the mounting geometry lives in the connect joint (built from triorb.urdf's `mount`:
  // z=0.60, Rz(-90deg)), so there is nothing left to keep in sync.
  //
  // The old "posW() BEFORE addContact()" rule no longer applies: it existed because a
  // contact captures its frame at add time (mc_rtc's mobile-arm-controller tutorial), and
  // there are no contacts any more. posW() must still precede the task reset()s below,
  // since those seed their targets from the current pose.
  robots().robot(0).posW(sva::PTransformd::Identity());

  // 2. WBC tasks. They all address the merged robot (index 0). Names carry the part they
  // act on: callm_ = whole robot, ur5e_ = arm joints only, triorb_ = base joints only.
  //
  // The end-effector task is callm_ because it spans the whole robot: the Tool surface is
  // on the arm, but the QP is free to reach its target with the base as much as the arm --
  // that is the point of merging them.
  callmEeTask_ = std::make_shared<mc_tasks::SurfaceTransformTask>("Tool", robots(), 0, callmEeStiffness_,
                                                                  callmEeWeight_);
  callmEeTask_->name("callm_ee");
  callmEeTask_->reset(); // target := current Tool pose
  solver().addTask(callmEeTask_);

  triorbBaseTask_ =
      std::make_shared<mc_tasks::TransformTask>("base", robots(), 0, triorbBaseStiffness_, triorbBaseWeight_);
  triorbBaseTask_->name("triorb_base");
  triorbBaseTask_->reset(); // target := current base pose
  solver().addTask(triorbBaseTask_); // gains are (re)applied every tick, see applyCommandsToTasks
  triorbBaseTargetPose_ = triorbBaseTask_->target();

  // Light posture regularizing the base joints only. Base joint limits already come from
  // the merged robot's kinematicsConstraint, so no extra KinematicsConstraint is needed.
  //
  // All the posture tasks share robot 0 -- they are ordinary weighted objectives over
  // disjoint joint sets -- but they must be renamed: PostureTask derives its name from the
  // robot name, so they would all be "posture_callm" and collide in the log/GUI. The name
  // has to be set before addTask().
  triorbPostureTask_ =
      std::make_shared<mc_tasks::PostureTask>(solver(), 0, triorbPostureStiffness_, triorbPostureWeight_);
  triorbPostureTask_->name("triorb_posture");
  triorbPostureTask_->selectActiveJoints(solver(), baseJointNames_);
  solver().addTask(triorbPostureTask_);

  // 3. Robotiq gripper: connect() already bolted it onto the tool, so all that is left is
  // to regularize its knuckle joints (a fixed regularizer, not exposed to WbcData). Named
  // gripper_ to match the prefix its joints carry in the merged robot.
  if(gripperEnabled_)
  {
    gripperPostureTask_ =
        std::make_shared<mc_tasks::PostureTask>(solver(), 0, gripperPostureStiffness_, gripperPostureWeight_);
    gripperPostureTask_->name("gripper_posture");
    gripperPostureTask_->selectActiveJoints(solver(), gripperJointNames_);
    solver().addTask(gripperPostureTask_);
  }

  // 4. Arm standby posture (resolves arm redundancy in the null space of the EE task).
  // postureTask was named and restricted to the arm joints in the constructor; both survive
  // reset (MCController::reset only re-seeds the posture target, not the dimWeight).
  postureTask->stiffness(ur5ePostureStiffness_);
  postureTask->weight(ur5ePostureWeight_);
  const std::vector<double> armStandby = {0.0, -M_PI / 2, M_PI / 2, 0.0, 0.0, 0.0};

  // 5. Seed the commanded targets so the robot holds still until a commander moves them.
  datastore().assign<sva::PTransformd>(CALLM_EE_TARGET_KEY, callmEeTask_->target());
  datastore().assign<sva::PTransformd>(TRIORB_BASE_TARGET_KEY, triorbBaseTask_->target());
  datastore().assign<Eigen::Vector3d>(TRIORB_BASE_VELOCITY_KEY, Eigen::Vector3d::Zero());
  datastore().assign<std::vector<double>>(UR5E_POSTURE_KEY, armStandby);
  datastore().assign<std::vector<double>>(TRIORB_POSTURE_KEY, std::vector<double>(baseJointNames_.size(), 0.0));
  datastore().assign<double>(GRIPPER_OPENING_KEY, 0.0);
  // Each episode starts from the YAML-default gains; the client picks its mode/compliance
  // via the WbcData task_weights / task_stiffness / task_damping_ratio fields. Damping is
  // seeded critical (zeta = 1) for every task; shape it at runtime via task_damping_ratio.
  datastore().assign<std::vector<double>>(
      TASK_WEIGHTS_KEY, std::vector<double>{callmEeWeight_, ur5ePostureWeight_, triorbBaseWeight_, triorbPostureWeight_});
  datastore().assign<std::vector<double>>(
      TASK_STIFFNESS_KEY,
      std::vector<double>{callmEeStiffness_, ur5ePostureStiffness_, triorbBaseStiffness_, triorbPostureStiffness_});
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
  // Arm and base now live in the same (merged) robot.
  const auto & ur5e = robot();
  const sva::PTransformd toolPose = ur5e.surfacePose("Tool");
  m.eef_pos = {toolPose.translation().x(), toolPose.translation().y(), toolPose.translation().z()};
  m.eef_quat = wireQuat(toolPose); // standard ROS/Hamilton (see wireQuat)
  for(size_t i = 0; i < armJointNames_.size(); ++i)
  {
    m.posture_arm[i] = ur5e.mbc().q[ur5e.jointIndexByName(armJointNames_[i])][0];
  }

  const auto & tri = robot();
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
