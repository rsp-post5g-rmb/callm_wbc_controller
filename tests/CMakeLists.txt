file(
  GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/etc/mc_rtc.yaml"
  INPUT "${CMAKE_CURRENT_SOURCE_DIR}/etc/mc_rtc.yaml")

# Make the controller configuration available next to the built module so the
# framework can find it during the test run.
configure_file("${PROJECT_SOURCE_DIR}/etc/CallmWbcController.in.yaml"
               "${CMAKE_CURRENT_BINARY_DIR}/etc/CallmWbcController.cmake.yaml")
file(
  GENERATE
  OUTPUT "$<TARGET_FILE_DIR:CallmWbcController>/etc/CallmWbcController.yaml"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/etc/CallmWbcController.cmake.yaml")

if(MSVC AND _VCPKG_INSTALLED_DIR)
  set(VCPKG_INSTALL_PREFIX "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_PREFIX}")
  # Only debug actually exists but we make sure it's first in that case
  set(CMAKE_MSVCIDE_RUN_PATH
      "${VCPKG_INSTALL_PREFIX}/\$(Configuration)/bin;${VCPKG_INSTALL_PREFIX}/bin;${CMAKE_MSVCIDE_RUN_PATH}"
  )
endif()

add_executable(test_controller main.cpp)
target_link_libraries(test_controller PUBLIC mc_rtc::mc_control)
add_dependencies(test_controller CallmWbcController)
add_test(NAME test_controller
         COMMAND test_controller
                 ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/etc/mc_rtc.yaml)
