# Install script for directory: /home/korivi/model_car/ecl_lib/src/ecl_containers/include/ecl/containers

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecl/containers" TYPE FILE FILES
    "/home/korivi/model_car/ecl_lib/src/ecl_containers/include/ecl/containers/array.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_containers/include/ecl/containers/definitions.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_containers/include/ecl/containers/stencil.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_containers/include/ecl/containers/initialiser.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_containers/include/ecl/containers/fifo.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_containers/include/ecl/containers/converters.hpp"
    "/home/korivi/model_car/ecl_lib/src/ecl_containers/include/ecl/containers/push_and_pop.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/korivi/model_car/ecl_lib/build/ecl_containers/include/ecl/containers/common/cmake_install.cmake")
  include("/home/korivi/model_car/ecl_lib/build/ecl_containers/include/ecl/containers/array/cmake_install.cmake")
  include("/home/korivi/model_car/ecl_lib/build/ecl_containers/include/ecl/containers/push_and_pop/cmake_install.cmake")
  include("/home/korivi/model_car/ecl_lib/build/ecl_containers/include/ecl/containers/stencil/cmake_install.cmake")

endif()

