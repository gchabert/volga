cmake_minimum_required(VERSION 2.8.3)
project(volga_core)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_compile_options(-g)
#add_compile_options(-O0)


find_package(catkin REQUIRED COMPONENTS
 roscpp
 roslib
 urdf
 tf
 tf_conversions
 kdl_parser
 trac_ik_lib
)

set(VOLGA_CORE_LIB_NAME volga_core)

pkg_search_module(IBEX REQUIRED ibex)
pkg_search_module(OPENMESH REQUIRED openmesh)


catkin_package(
  INCLUDE_DIRS include ${IBEX_INCLUDE_DIRS} ${OPENMESH_INCLUDE_DIRS}
  LIBRARIES ${VOLGA_CORE_LIB_NAME}
  CATKIN_DEPENDS roscpp roslib urdf tf tf_conversions kdl_parser trac_ik_lib
  DEPENDS ${IBEX} ${OPENMESH}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${IBEX_INCLUDE_DIRS}
 ${OPENMESH_INCLUDE_DIRS}
)

link_directories(${IBEX_LIBRARY_DIRS})
link_directories(${OPENMESH_LIBRARY_DIRS})

## Declare a C++ library
add_library(${VOLGA_CORE_LIB_NAME}
	src/Point3D.cpp 
	src/Triangle.cpp 
	src/Ball.cpp 
	src/Rotation.cpp
	src/Frame.cpp 
	src/Mesh.cpp 
	src/KinematicModel.cpp 
	src/LocalPlanner.cpp 
	src/FrameInterpol.cpp 
	src/tf2ibex.cpp 
	src/Quaternion.cpp 
	src/UnitQuaternion.cpp 
	src/KDLRobot.cpp 
	src/SerialRobot.cpp 
	src/FKInterpol.cpp
)
target_link_libraries(${VOLGA_CORE_LIB_NAME} 
	${catkin_LIBRARIES} ${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES}
) 


#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${VOLGA_CORE_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
	add_executable(test_undirected_graph tests/test_undirected_graph.cpp include/volga_core/UndirectedGraph.h)
	add_executable(test_serial_robot tests/test_serial_robot.cpp )
	add_executable(test_frame_interpol tests/test_frame_interpol.cpp )
	target_link_libraries(test_undirected_graph ${catkin_LIBRARIES} ${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES})
	target_link_libraries(test_serial_robot ${catkin_LIBRARIES} ${VOLGA_CORE_LIB_NAME} ${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES})
	target_link_libraries(test_frame_interpol ${catkin_LIBRARIES} ${VOLGA_CORE_LIB_NAME} ${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES})
endif()

