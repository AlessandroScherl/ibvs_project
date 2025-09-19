# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ibvs_simulation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ibvs_simulation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ibvs_simulation_FOUND FALSE)
  elseif(NOT ibvs_simulation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ibvs_simulation_FOUND FALSE)
  endif()
  return()
endif()
set(_ibvs_simulation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ibvs_simulation_FIND_QUIETLY)
  message(STATUS "Found ibvs_simulation: 0.0.1 (${ibvs_simulation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ibvs_simulation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ibvs_simulation_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ibvs_simulation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ibvs_simulation_DIR}/${_extra}")
endforeach()
