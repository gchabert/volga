cmake_minimum_required(VERSION 2.8.3)
project(volga_examples)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_compile_options(-g)
#add_compile_options(-O0)

## Find catkin macros and libraries
## if COMPONENTS list like find	_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp
 roslib
 urdf
 tf
 tf_conversions
 volga_core
)

pkg_search_module(IBEX REQUIRED ibex)
pkg_search_module(OPENMESH REQUIRED openmesh)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 ${catkin_INCLUDE_DIRS}
 ${IBEX_INCLUDE_DIRS}
 ${OPENMESH_INCLUDE_DIRS}
)

link_directories(${IBEX_LIBRARY_DIRS})
link_directories(${OPENMESH_LIBRARY_DIRS})


add_executable(kinematic_path src/kinematic_path.cpp)
target_link_libraries(kinematic_path ${catkin_LIBRARIES} ${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES})

add_executable(tree_mesh src/tree_mesh.cpp)

target_link_libraries(tree_mesh ${catkin_LIBRARIES} ${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES})

add_executable(rigid_body_path src/rigid_body_path.cpp)
target_link_libraries(rigid_body_path ${catkin_LIBRARIES} ${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES})

add_executable(ik_test src/ik_test.cpp)
target_link_libraries(ik_test ${catkin_LIBRARIES} ${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES})


#############
## Install ##
#############

install(TARGETS kinematic_path tree_mesh rigid_body_path ik_test
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY cfg launch meshes urdf
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

