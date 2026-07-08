#include <mc_control/mc_global_controller.h>

#include <cstdlib> // std::_Exit

// NOTE on shutdown: mc_rtc's ROS plugin keeps a process-wide static singleton
// (ROSBridgeImpl) whose RobotPublishers connect a Slot to Robots::onRobotRemoved().
// That static is destroyed at process exit -- AFTER the local MCGlobalController (and
// its Robots signal) is already gone -- so its ~Slot calls Signal::disconnect() on a
// freed Robots, a use-after-free that segfaults during normal `return` teardown.
// (Violates the SignalSlot contract: the emitter must outlive its subscribers.) The
// bug is in mc_rtc, not this controller; the controller runs the smoke test to
// completion. We therefore end the test with std::_Exit, which skips both local and
// static destructors and avoids the dangling-signal teardown path entirely.

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    mc_rtc::log::critical("Should provide a configuration file for mc_rtc");
    return 1;
  }
  mc_rtc::log::critical("LOADING {}", argv[1]);
  mc_control::MCGlobalController gc(argv[1]);

  const auto & mb = gc.robot().mb();
  const auto & mbc = gc.robot().mbc();
  const auto & rjo = gc.ref_joint_order();
  std::vector<double> initq;
  for(const auto & jn : rjo)
  {
    for(const auto & qi : mbc.q[static_cast<unsigned int>(mb.jointIndexByName(jn))])
    {
      initq.push_back(qi);
    }
  }

  std::vector<double> qEnc(initq.size(), 0);
  std::vector<double> alphaEnc(initq.size(), 0);
  auto simulateSensors = [&]()
  {
    auto & robot = gc.robot();
    for(unsigned i = 0; i < robot.refJointOrder().size(); i++)
    {
      auto jIdx = robot.jointIndexInMBC(i);
      if(jIdx != -1)
      {
        auto jointIndex = static_cast<unsigned>(jIdx);
        qEnc[i] = robot.mbc().q[jointIndex][0];
        alphaEnc[i] = robot.mbc().alpha[jointIndex][0];
      }
    }
    gc.setEncoderValues(qEnc);
    gc.setEncoderVelocities(alphaEnc);
    if(gc.robot().hasBodySensor("FloatingBase"))
    {
      gc.setSensorPositions({{"FloatingBase", robot.posW().translation()}});
      gc.setSensorOrientations({{"FloatingBase", Eigen::Quaterniond{robot.posW().rotation()}}});
    }
  };

  gc.setEncoderValues(qEnc);
  gc.init(initq, gc.robot().module().default_attitude());
  gc.running = true;

  for(size_t i = 0; i < 10; ++i)
  {
    simulateSensors();
    if(!gc.run())
    {
      std::cout.flush();
      std::_Exit(1); // skip the mc_rtc ROS-plugin static-teardown UAF (see note above)
    }
  }

  std::cout.flush();
  std::_Exit(0); // controller ran to completion; skip destructors to avoid the teardown UAF
}
