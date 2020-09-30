cmake_minimum_required(VERSION 2.8.3)
project(volga_moveit)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 roslib
 urdf
 volga_core
 tf_conversions
 moveit_core
 moveit_ros_move_group
 moveit_visual_tools
 moveit_planners_ompl
 moveit_ros_planning_interface
)

find_package(ompl REQUIRED)

pkg_search_module(IBEX REQUIRED ibex)
pkg_search_module(OPENMESH REQUIRED openmesh)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp roslib urdf volga_core tf_conversions 
 				moveit_core moveit_ros_move_group moveit_visual_tools moveit_planners_ompl  
 				moveit_ros_planning_interface
 DEPENDS ${IBEX} ${OPENMESH} OMPL
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${IBEX_INCLUDE_DIRS}
 ${OPENMESH_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
)

link_directories(${IBEX_LIBRARY_DIRS})
link_directories(${OPENMESH_LIBRARY_DIRS})

# Volga planning plugin
add_library(moveit_volga_planner_plugin 
	src/volga_planner_manager.cpp
	src/motion_validator.cpp
)

target_link_libraries(
	moveit_volga_planner_plugin 
	${catkin_LIBRARIES} 
	${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES} ${OMPL_LIBRARIES} 
) 

set_target_properties(moveit_volga_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

#############
## Install ##
#############

install(
  TARGETS
    moveit_volga_planner_plugin
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
 install(
	DIRECTORY 
		include
	DESTINATION 
		${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES volga_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
	add_executable(test1 tests/test1.cpp)
	add_dependencies(test1 ${volga_moveit_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(
		test1 moveit_volga_planner_plugin ${catkin_LIBRARIES}  
		${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES}
	)
	target_compile_options(test1 PUBLIC ${IBEX_CFLAGS_OTHER} ${OPENMESH_CFLAGS_OTHER})
	
	add_executable(test2 tests/test2.cpp)
	add_dependencies(test2 ${volga_moveit_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(
		test2 moveit_volga_planner_plugin ${catkin_LIBRARIES}  
		${IBEX_LIBRARIES} ${OPENMESH_LIBRARIES}
	)
	target_compile_options(test2 PUBLIC ${IBEX_CFLAGS_OTHER} ${OPENMESH_CFLAGS_OTHER})
	
	install(TARGETS test1 test2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

