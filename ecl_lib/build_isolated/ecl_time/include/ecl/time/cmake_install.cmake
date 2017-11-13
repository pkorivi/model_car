# Install script for directory: /home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/korivi/model_car/ecl_lib/install_isolated")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecl/time" TYPE FILE FILES
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/frequency.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/duration.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/snooze.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/timestamp_win.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/sleep.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/stopwatch.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/random_number_generator.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/sleep_pos.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/timestamp.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/timestamp_pos.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/snooze_win.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/timestamp_base.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/time_data.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/cpuwatch.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/macros.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/sleep_win.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/cpuwatch_rt.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_time/include/ecl/time/snooze_pos.hpp"
    )
endif()

