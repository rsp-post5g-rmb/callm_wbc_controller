cmake_minimum_required(VERSION 3.22)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME CallmWbcController)
set(PROJECT_DESCRIPTION "Whole-body controller for a UR5e arm on a TriOrb base")
set(PROJECT_URL "")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

# Built either standalone (find_package) or inside mc_rtc / the superbuild.
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

# Install the controller configuration next to the other mc_rtc controller configs.
configure_file(etc/CallmWbcController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/CallmWbcController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/CallmWbcController.yaml"
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
