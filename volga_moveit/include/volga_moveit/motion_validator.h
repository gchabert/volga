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

#ifndef __VOLGA_MOVEIT_MOTION_VALIDATOR__
#define __VOLGA_MOVEIT_MOTION_VALIDATOR__

#include <ompl/base/MotionValidator.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <volga_core/LocalPlanner.h>

namespace volga_moveit {

OMPL_CLASS_FORWARD(MotionValidator);

class MotionValidator : public ompl::base::MotionValidator {
public:
	/** \brief See OMPL documentation */
	MotionValidator(ompl_interface::ModelBasedPlanningContextPtr& context, volga_core::SerialRobot& volga_model, std::vector<volga_core::Mesh*>& obstacles);

	/** \brief See OMPL documentation */
	//MotionValidator(ompl_interface::ModelBasedPlanningContextPtr& context, volga_core::SerialRobot& volga_model, std::vector<volga_core::Mesh*>& obstacles);

	/** \brief See OMPL documentation */
	virtual ~MotionValidator();

	/** \brief See OMPL documentation */
	virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;

	/** \brief See OMPL documentation */
	virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const;

protected:
	bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double>* lastValid) const;

	const moveit::core::JointModelGroup* group;
	const moveit::core::RobotState& initial_state;
	const ompl_interface::ModelBasedStateSpacePtr state_space;
	mutable moveit::core::RobotState state1;
	mutable moveit::core::RobotState state2;
	volga_core::SerialRobot& volga_model;
	std::vector<volga_core::Mesh*>& obstacles;
};

} // end namespace volga_moveit

#endif
