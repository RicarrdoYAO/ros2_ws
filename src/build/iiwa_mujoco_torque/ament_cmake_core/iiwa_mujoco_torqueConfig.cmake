# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_iiwa_mujoco_torque_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED iiwa_mujoco_torque_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(iiwa_mujoco_torque_FOUND FALSE)
  elseif(NOT iiwa_mujoco_torque_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(iiwa_mujoco_torque_FOUND FALSE)
  endif()
  return()
endif()
set(_iiwa_mujoco_torque_CONFIG_INCLUDED TRUE)

# output package information
if(NOT iiwa_mujoco_torque_FIND_QUIETLY)
  message(STATUS "Found iiwa_mujoco_torque: 0.0.0 (${iiwa_mujoco_torque_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'iiwa_mujoco_torque' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT iiwa_mujoco_torque_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(iiwa_mujoco_torque_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${iiwa_mujoco_torque_DIR}/${_extra}")
endforeach()
