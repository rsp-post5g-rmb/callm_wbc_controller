#include "CallmWbcController.h"

#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Transform.h>

#include <cmath>

CallmWbcController::CallmWbcController(mc_rbdyn::RobotModulePtr rm,
                                      double dt,
                                      const mc_rtc::Configuration & config)
// robot 0: UR5e (floating base, from MainRobot: UR5eFloatingBase)
// robot 1: triorb (the omnidirectional base; fixed root + base_x/base_y/base_yaw joints)
// robot 2: env/ground (visual reference only)
: mc_control::MCController({rm, mc_rbdyn::RobotLoader::get_robot_module("triorb"),
                            mc_rbdyn::RobotLoader::get_robot_module("env/ground")},
                           dt)
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

  // ---- Constraints + main-robot posture ------------------------------------
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint); // UR5e (robot 0) joint limits
  solver().addConstraintSet(selfCollisionConstraint); // UR5e self-collisions
  solver().addTask(postureTask); // UR5e posture (robot 0)
  solver().setContacts({}); // start with no contacts; the arm<->base contact is added in reset()

  // ---- Arm <-> base attachment surface -------------------------------------
  // The TriOrb module ships no RSDF surfaces, so we declare, at runtime, a planar
  // surface on its `mount` link that mirrors the UR5e "Base" surface. The arm's
  // floating base is contacted onto this surface in reset(). The points match the
  // UR5e base footprint (see mc_ur5e_description/rsdf/ur5e/base_link.rsdf).
  std::vector<std::pair<double, double>> mountPoints = {
      {-0.0745, -0.0745}, {0.0745, -0.0745}, {0.0745, 0.0745}, {-0.0745, 0.0745}};
  robot("triorb").addSurface(std::make_shared<mc_rbdyn::PlanarSurface>(
      armMountSurface_, "mount", sva::PTransformd::Identity(), "plastic", mountPoints));

  // Note: no dof-masked base<->ground contact is needed. Unlike Dingo in the
  // mobile-arm tutorial, the TriOrb module is fixed-base with explicit planar
  // joints (base_x / base_y / base_yaw), so its planar motion is intrinsic to the
  // kinematics and is solved directly by the QP through those joints.

  // ---- Arm <-> base collision avoidance ------------------------------------
  // mc_rtc auto-builds an sch::S_Box collision convex named "base" for the TriOrb
  // base link directly from the URDF <box> collision primitive, so no extra hull
  // file is needed. We guard the distal arm links against that box. base_link and
  // shoulder_link are intentionally excluded: the arm base is rigidly mounted on
  // top of the base, so they sit permanently against it by design. (UR5e internal
  // self-collisions are already covered by selfCollisionConstraint.)
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

  mc_rtc::log::success("CallmWbcController init done");
}

void CallmWbcController::setupTargetsIO()
{
  if(ioReady_) { return; }
  ioReady_ = true;

  // Datastore is the single source of truth for the commanded targets. The GUI
  // reads/writes these keys, and an external commander can assign() them live.
  datastore().make<sva::PTransformd>(EE_TARGET_KEY, sva::PTransformd::Identity());
  datastore().make<sva::PTransformd>(BASE_TARGET_KEY,
                                     sva::PTransformd(Eigen::Vector3d(0.0, 0.0, baseHeight_)));

  gui()->addElement(
      {"CallmWbc"},
      mc_rtc::gui::Transform(
          "EE target [world]", [this]() -> sva::PTransformd
          { return datastore().get<sva::PTransformd>(EE_TARGET_KEY); },
          [this](const sva::PTransformd & p) { datastore().assign<sva::PTransformd>(EE_TARGET_KEY, p); }),
      mc_rtc::gui::Transform(
          "Base target [world]", [this]() -> sva::PTransformd
          { return datastore().get<sva::PTransformd>(BASE_TARGET_KEY); },
          [this](const sva::PTransformd & p) { datastore().assign<sva::PTransformd>(BASE_TARGET_KEY, p); }),
      mc_rtc::gui::ArrayInput(
          "Base target [x, y, yaw]", {"x", "y", "yaw"},
          [this]() -> Eigen::Vector3d
          {
            const auto & X = datastore().get<sva::PTransformd>(BASE_TARGET_KEY);
            const auto & R = X.rotation();
            return {X.translation().x(), X.translation().y(), std::atan2(R(0, 1), R(0, 0))};
          },
          [this](const Eigen::Vector3d & xyyaw)
          {
            datastore().assign<sva::PTransformd>(
                BASE_TARGET_KEY,
                sva::PTransformd(sva::RotZ(xyyaw.z()), Eigen::Vector3d(xyyaw.x(), xyyaw.y(), baseHeight_)));
          }),
      mc_rtc::gui::Button("Sync targets to current robot",
                          [this]()
                          {
                            datastore().assign<sva::PTransformd>(EE_TARGET_KEY,
                                                                 robots().robot(0).surfacePose("Tool"));
                            datastore().assign<sva::PTransformd>(BASE_TARGET_KEY,
                                                                 robots().robot("triorb").bodyPosW("base"));
                          }));
}

bool CallmWbcController::run()
{
  // Pull the commanded targets and step the QP. Nothing else belongs here.
  if(eeTask_) { eeTask_->target(datastore().get<sva::PTransformd>(EE_TARGET_KEY)); }
  if(baseTask_) { baseTask_->target(datastore().get<sva::PTransformd>(BASE_TARGET_KEY)); }
  return mc_control::MCController::run();
}

void CallmWbcController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  // 1. Initial poses, set BEFORE the coupling contact is added. The TriOrb root
  // (`world`) stays at the origin; the base is carried by its base_x/base_y/base_yaw
  // joints. The arm's floating base is placed onto the mount link so that the UR5e
  // "Base" surface coincides with the runtime-added "ArmMount" surface.
  robots().robot("triorb").posW(sva::PTransformd::Identity());
  robots().robot(0).posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.0, mountHeight_)));

  // 2. Rigid arm<->base attachment (all 6 dof constrained). Added AFTER posW so the
  // contact frame captures the intended relative pose (see the ordering note in the
  // mobile-arm tutorial).
  addContact({"triorb", "ur5e", armMountSurface_, "Base"});

  // 3. WBC tasks.
  eeTask_ = std::make_shared<mc_tasks::SurfaceTransformTask>("Tool", robots(), 0, eeStiffness_, eeWeight_);
  eeTask_->reset(); // target := current Tool pose
  solver().addTask(eeTask_);

  baseTask_ = std::make_shared<mc_tasks::TransformTask>("base", robots(), 1, baseStiffness_, baseWeight_);
  baseTask_->reset(); // target := current base pose
  solver().addTask(baseTask_);

  // Per-robot constraint + light posture for the base (mirrors the articulated-object
  // handling in the mobile-arm tutorial).
  triorbKinematics_ = std::make_unique<mc_solver::KinematicsConstraint>(robots(), 1, solver().dt());
  solver().addConstraintSet(triorbKinematics_);
  triorbPostureTask_ =
      std::make_shared<mc_tasks::PostureTask>(solver(), 1, triorbPostureStiffness_, triorbPostureWeight_);
  solver().addTask(triorbPostureTask_);

  // UR5e standby posture to resolve arm redundancy in the null space of the EE task.
  postureTask->stiffness(ur5ePostureStiffness_);
  postureTask->weight(ur5ePostureWeight_);
  postureTask->target({{"shoulder_lift_joint", {-M_PI / 2}}, {"elbow_joint", {M_PI / 2}}});

  // 4. Seed the commanded targets with the current task targets so the robot holds
  // still until a commander (GUI / datastore) moves them.
  datastore().assign<sva::PTransformd>(EE_TARGET_KEY, eeTask_->target());
  datastore().assign<sva::PTransformd>(BASE_TARGET_KEY, baseTask_->target());

  mc_rtc::log::success("CallmWbcController reset done");
}

CONTROLLER_CONSTRUCTOR("CallmWbcController", CallmWbcController)
