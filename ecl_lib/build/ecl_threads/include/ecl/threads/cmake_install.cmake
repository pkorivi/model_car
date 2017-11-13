# Install script for directory: /home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecl/threads" TYPE FILE FILES
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/mutex.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/thread_exceptions_pos.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/mutex_pos.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/thread_pos.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/threadable.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/thread_win.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/priority_win.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/priority_common.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/mutex_w32.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/thread.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/macros.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/priority_pos.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_threads/include/ecl/threads/priority.hpp"
    )
endif()

