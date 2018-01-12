#cmake_minimum_required(VERSION 2.8.3)

cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(autonomos_obstacle_msgs)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
#autonomos_common_types
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	message_generation
	nav_msgs
	nodelet
	std_msgs
	visualization_msgs
)

# include common CMake setup
# find_package(autonomos_cmake REQUIRED)
# include(Autonomos)

# add message generation
add_message_files(
	DIRECTORY
		msg
	FILES
		Obstacle.msg
		Obstacles.msg
)

generate_messages(
	DEPENDENCIES
		geometry_msgs
		nav_msgs
		std_msgs
		visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES
	CATKIN_DEPENDS
		geometry_msgs
		message_runtime
		nav_msgs
		std_msgs
		visualization_msgs
#	DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(SYSTEM
	${catkin_INCLUDE_DIRS}
)
include_directories(
        "src/"
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_visualizer
	src/visualization/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_visualizer
	${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_visualizer_nodelet
	src/visualization/ObstacleVisualizer.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(${PROJECT_NAME}_visualizer
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_visualizer_nodelet
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_visualizer_nodelet
	${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(
	TARGETS
		${PROJECT_NAME}_visualizer
		${PROJECT_NAME}_visualizer_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
	FILES nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############
