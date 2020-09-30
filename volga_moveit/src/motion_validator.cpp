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

#include <volga_moveit/motion_validator.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <mutex> // ibex does not support multi-threading

using namespace std;

namespace volga_moveit {

namespace {
mutex mtx;

double check_motion_locked(volga_core::SerialRobot& volga_model, int i, volga_core::LocalPlanner::SpaceType space, volga_core::Mesh& obstacle, const ibex::Vector& q0, const ibex::Vector& q1) {
	mtx.lock();
	volga_core::LocalPlanner planner(volga_model, i, space, obstacle, q0, q1);
	planner.update_mesh = false;
	//planner.log = true;
	double topt = planner.max_time(volga_model.epsilon);
	mtx.unlock();
	return topt;
}

}

MotionValidator::MotionValidator(ompl_interface::ModelBasedPlanningContextPtr& context, volga_core::SerialRobot& volga_model, std::vector<volga_core::Mesh*>& obstacles)
: ompl::base::MotionValidator(context->getOMPLSimpleSetup()->getSpaceInformation()),
  group(context->getJointModelGroup()),
  initial_state(context->getCompleteInitialRobotState()),
  state_space(context->getOMPLStateSpace()),
  state1(context->getRobotModel()),
  state2(context->getRobotModel()),
  volga_model(volga_model), obstacles(obstacles) {

	const vector<const moveit::core::JointModel*>& joint_models = context->getRobotModel()->getJointModels();
	for (vector<const moveit::core::JointModel*>::const_iterator it=joint_models.begin(); it!=joint_models.end(); ++it) {
		if ((*it)->getVariableCount()>1) {
			cout << (*it)->getName() << " index=" << (*it)->getJointIndex() << " " << (*it)->getTypeName() << endl;
			ROS_ERROR("[volga_moveit] Multi-DOF joints not implemented");
		}
	}
}

MotionValidator::~MotionValidator() {

}

bool MotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
	return checkMotion(s1,s2,NULL);
}

bool MotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
	return checkMotion(s1,s2,&lastValid);
}

bool MotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double>* lastValid) const {
	const ompl_interface::ModelBasedStateSpace::StateType* _s1 = dynamic_cast<const ompl_interface::ModelBasedStateSpace::StateType*>(s1);
	const ompl_interface::ModelBasedStateSpace::StateType* _s2 = dynamic_cast<const ompl_interface::ModelBasedStateSpace::StateType*>(s2);
	if (_s1==NULL || _s2==NULL) {
		ROS_ERROR("Cannot convert OMPL state to Moveit! state");
		return ompl::base::MotionValidator::checkMotion(s1,s2);
	}

	state_space->copyToRobotState(state1, _s1);
	state_space->copyToRobotState(state2, _s2);

	uint n=volga_model.fk.nb_actuated;
	ibex::Vector q0(n);
	ibex::Vector q1(n);

	for (uint i=0; i<n; i++) {
		const string& name=volga_model.fk.joint_name(i);
		if (group && !group->hasJointModel(name)) {
			// set inactive joints to their initial state values
			// (I suppose this is the right way to do....)
			q0[i] = initial_state.getJointPositions(name)[0]; // [0] because single-DOF joint
			q1[i] = initial_state.getJointPositions(name)[0];
		} else {
			q0[i] = state1.getJointPositions(name)[0]; // [0] because single-DOF joint
			q1[i] = state2.getJointPositions(name)[0];
		}
	}

	double topt = 1;
	uint link_hit = 0; // by default
	for (uint i=0; i<volga_model.fk.nb_links; i++) {
		if (!volga_model.has_collision(i)) continue;
		for (uint j=0; j<obstacles.size(); j++) {
			double topt_i_j = check_motion_locked(volga_model, i, volga_core::LocalPlanner::JOINT, *obstacles[j], q0, q1);
			if (topt_i_j<topt) {
				topt=topt_i_j;
				link_hit = i;
			}
		}
	}

	if (topt!=1) {
		ROS_WARN("Volga detected an obstacle!");
		if (lastValid) {
			ompl_interface::ModelBasedStateSpace::StateType* _st = dynamic_cast<ompl_interface::ModelBasedStateSpace::StateType*>(lastValid->first);
			if (_st==NULL) {
				ROS_ERROR("Cannot convert OMPL \"last valid\" state to Moveit! state");
			} else {
				ibex::Vector qt = (1-topt)*q0+topt*q1; // note: we neglect rounding errors
				for (uint i=0; i<n; i++)
					_st->values[i] = qt[i];
			}
			lastValid->second = topt;
		}
	}

	return topt==1;
}

} // end namespace volga_moveit

