# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_daemon_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED daemon_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(daemon_FOUND FALSE)
  elseif(NOT daemon_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(daemon_FOUND FALSE)
  endif()
  return()
endif()
set(_daemon_CONFIG_INCLUDED TRUE)

# output package information
if(NOT daemon_FIND_QUIETLY)
  message(STATUS "Found daemon: 0.1.0 (${daemon_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'daemon' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${daemon_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(daemon_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${daemon_DIR}/${_extra}")
endforeach()
