# Install script for directory: /home/korivi/model_car/ecl_lib/src/ecl_sigslots_lite/include/ecl/sigslots_lite

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/korivi/model_car/ecl_lib/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecl/sigslots_lite" TYPE FILE FILES
    "/home/korivi/model_car/ecl_lib/src/ecl_sigslots_lite/include/ecl/sigslots_lite/connect.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_sigslots_lite/include/ecl/sigslots_lite/utilities.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_sigslots_lite/include/ecl/sigslots_lite/signal.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_sigslots_lite/include/ecl/sigslots_lite/errors.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_sigslots_lite/include/ecl/sigslots_lite/slot.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_sigslots_lite/include/ecl/sigslots_lite/managers.hpp"
    )
endif()

