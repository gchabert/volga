/*************************************************************************************
 *
 * Copyright (c) 2020, IRT Jules Verne.
 * www.irt-jules-verne.fr
 *
 * Author: Gilles Chabert
 *
 * ---------------------------------------------------------------
 * This work was partially funded by the ROSIN European project
 *           www.rosin-project.eu
 * ---------------------------------------------------------------
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <eigen3/Eigen/src/Geometry/Transform.h>

#include <volga_core/KinematicModel.h>
#ifndef  __MOVEIT_H__
#define  __MOVEIT_H__
#include <visualization_msgs/Marker.h>

// to be used with:
//      roslaunch panda_moveit_config demo.launch

int main(int argc, char** argv) {

	ros::init(argc, argv, "moveit_test");

	ros::NodeHandle node;

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	volga_core::KinematicModel volga_model;
	volga_model.init(*kinematic_model->getURDF(),true);

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
	  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	kinematic_state->setToRandomPositions(joint_model_group);
	const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link7");

	/* Print end-effector pose. Remember that this is in the model frame */
	ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
	ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
}

#endif /* __VOLGA_CORE_MOVEIT_H__ */
