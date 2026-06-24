#pragma once

#include <mc_control/mc_controller.h>

#include <mc_solver/KinematicsConstraint.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/TransformTask.h>

#include "api.h"

/** Whole-body controller for a UR5e arm mounted on a TriOrb omnidirectional base.
 *
 * A single QP coordinates the arm joints and the base planar motion jointly:
 *   - robot 0: UR5e (floating base, `MainRobot: UR5eFloatingBase`)
 *   - robot 1: triorb (the TriOrb base)
 *   - robot 2: env/ground (visualization only)
 *
 * The UR5e floating base is rigidly attached to the TriOrb `mount` link through a
 * Base<->Base contact, so when the QP moves the base the arm follows, and when the
 * end-effector task pulls the hand the solver distributes the motion across both the
 * arm joints and the base degrees of freedom. That is the whole-body coordination,
 * with no explicit inverse kinematics.
 *
 * Unlike the mobile-arm tutorial this is adapted from, there is no scripted phase
 * machine. The arm end-effector target and the base target are exposed through the
 * GUI and the datastore so an external commander (teleop / diffusion policy) can drive
 * them live. `run()` only copies the commanded targets into the tasks and steps the QP.
 */
struct CallmWbcController_DLLAPI CallmWbcController : public mc_control::MCController
{
  CallmWbcController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  /** Datastore keys an external commander can read/assign (sva::PTransformd, world frame). */
  static constexpr auto EE_TARGET_KEY = "CallmWbcController::eeTarget";
  static constexpr auto BASE_TARGET_KEY = "CallmWbcController::baseTarget";

private:
  /** Build the GUI elements and the datastore entries bound to the targets. */
  void setupTargetsIO();

  // WBC tasks
  std::shared_ptr<mc_tasks::SurfaceTransformTask> eeTask_; ///< UR5e end-effector (Tool surface)
  std::shared_ptr<mc_tasks::TransformTask> baseTask_; ///< TriOrb base body pose
  std::shared_ptr<mc_tasks::PostureTask> triorbPostureTask_; ///< regularizes the base joints
  std::unique_ptr<mc_solver::KinematicsConstraint> triorbKinematics_; ///< base joint limits

  // Task gains / weights (loaded from the controller configuration)
  double eeStiffness_ = 5.0;
  double eeWeight_ = 1000.0;
  double baseStiffness_ = 2.0;
  double baseWeight_ = 1000.0;
  double ur5ePostureStiffness_ = 10.0;
  double ur5ePostureWeight_ = 5.0;
  double triorbPostureStiffness_ = 1.0;
  double triorbPostureWeight_ = 1.0;

  // Geometry of the TriOrb description (world Z of the relevant links at the home pose).
  // base link sits at z = 0.30, mount link at z = 0.60 (see mc_triorb_description/urdf/triorb.urdf).
  double baseHeight_ = 0.30;
  double mountHeight_ = 0.60;

  // Names of the runtime-added attachment surface on the TriOrb mount link.
  std::string armMountSurface_ = "ArmMount";

  bool ioReady_ = false; ///< guards one-time GUI/datastore setup across resets
};
