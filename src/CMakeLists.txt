set(controller_SRC CallmWbcController.cpp)

set(controller_HDR CallmWbcController.h WbcData.h api.h)

# add_controller is provided by mc_rtc: it builds a shared controller module and
# links it against mc_rtc::mc_control. The UR5e / TriOrb / Robotiq gripper robot
# modules are loaded at runtime (plugins), so they are build-ordering dependencies
# handled by the superbuild AddProject DEPENDS, not link-time dependencies here.
add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
                                                 "-DCallmWbcController_EXPORTS")

# The ROS2 command client (rclcpp + std_msgs) is pulled in through mc_rtc's ROS
# plugin target, exactly like explicit_compliance_controller. rclcpp and std_msgs
# come transitively from mc_rtc_3rd_party::ROS, so no extra find_package is needed.
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin (mc_rtc::mc_rtc_ros) is required to build CallmWbcController")
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_rtc_ros)
