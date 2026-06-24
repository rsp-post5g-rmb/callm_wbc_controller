set(controller_SRC CallmWbcController.cpp)

set(controller_HDR CallmWbcController.h api.h)

# add_controller is provided by mc_rtc: it builds a shared controller module and
# links it against mc_rtc::mc_control. The UR5e and TriOrb robot modules are loaded
# at runtime (plugins), so they are build-ordering dependencies handled by the
# superbuild AddProject DEPENDS, not link-time dependencies of this target.
add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
                                                 "-DCallmWbcController_EXPORTS")
