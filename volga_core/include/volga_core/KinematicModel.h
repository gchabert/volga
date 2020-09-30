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

#ifndef __VOLGA_CORE_KINEMATIC_MODEL_H__
#define __VOLGA_CORE_KINEMATIC_MODEL_H__

#include <ibex.h>

#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <tf/LinearMath/Transform.h>

#include "Frame.h"

namespace volga_core {

/**
 * \brief Ibex function for a kinematic model.
 *
 * The argument of the function is a n-sized column
 * vector corresponding to the n joint positions.
 *
 * The function returns either a 4x3 matrix or
 * 7x1 vector representing the pose of the effector
 * (depending if quaternions are used or not).
 *
 * In the first case, the first 3x3 submatrix
 * is the orientation (as a rotation matrix) and
 * the last column is the position.
 *
 * In the second case, the first 4 components
 * is the orientation (quaternion) and the 3
 * last ones the position.
 *
 */
class KinematicModel {
public:

	/**
	 * \brief Build an uinitialized model.
	 *
	 * Must be followed by a call to init(...)
	 */
	KinematicModel();

	/**
	 * \brief Delete this.
	 */
	~KinematicModel();

	/**
	 * \brief Return the Ibex function for a given link.
	 *
	 * The order of links is that of the URDF model.
	 */
	const ibex::Function& operator()(uint link) const;

	/**
	 * \brief Return the Ibex function for a given link.
	 */
	const ibex::Function& operator()(const std::string& link_name) const;

	/**
	 * \brief Initialize the model.
	 *
	 * \param model            - the URDF mode of the robot
	 * \param use_quaternions  - do we use a quaternion or rotation matrix?
	 * \param log              - verbose if true
	 */
	void init(const urdf::ModelInterface& model, bool use_quaternions, bool log=false);

	/**
	 * \brief Inverse kinematics
	 *
	 * Only possible with a quaternion-based representation,
	 * whence the 7-size vector in argument.
	 */
	const ibex::CovSolverData& inverse(const ibex::IntervalVector& x);

	/*
	 * \brief Number of actuated joints
	 */
	const unsigned int nb_actuated;

	/*
	 * \brief Number of links
	 */
	const unsigned int nb_links;

	/*
	 * \brief Get the name of a link
	 */
	const std::string& link_name(uint link) const;

	/*
	 * \brief Get the number of a link
	 */
	uint link_number(const std::string& link_name) const;

	/*
	 * \brief Get the name of a joint
	 */
	const std::string& joint_name(uint joint) const;

	/*
	 * \brief Get the number of joint
	 */
	uint joint_number(const std::string& joint_name) const;

	/*
	 * \brief Name of the base link.
	 */
	const std::string base_name;

	/*
	 * \brief Name of the (auto-detected) tool
	 *
	 * Is considered here as "tool" the child link of
	 * the last actuated joint.
	 *
	 * Useful when planning in Cartesian mode only.
	 */
	std::string tool_name;

protected:
	/*
	 * Check with tf (for debug)
	 */
	tf::Transform check_with_tf(uint joint, const ibex::Vector& q) const;

	/*
	 * Forward kinematics for each link
	 */
	ibex::Function* fk_links;

	/*
	 * Map a link name to a link number
	 */
	std::vector<std::string> link2name;

	/*
	 * Map a link number to a link name
	 */
	ibex::SymbolMap<uint> name2link;

	/*
	 * Map a joint name to a joint number
	 */
	std::vector<std::string> joint2name;

	/*
	 * Map a joint number to a joint name
	 */
	ibex::SymbolMap<uint> name2joint;

	/**
	 * Joint limits
	 */
	ibex::IntervalVector joint_limits;

	/**
	 * Domain of the goal pose (for inverse kinematic)
	 */
	ibex::Domain dx;

	/**
	 * Inverse kinematic system
	 * with the first component dropped
	 * (built if required)
	 */
	ibex::System* ik_sys1;

	/**
	 * Inverse kinematic solver
	 * with the first component dropped
	 * (built if required)
	 */
	ibex::DefaultSolver* ik_solver1;

	/**
	 * Inverse kinematic system
	 * with the 2nd component dropped
	 * (built if required)
	 */
	ibex::System* ik_sys2;

	/**
	 * Inverse kinematic solver
	 * with the 2nd component dropped
	 * (built if required)
	 */
	ibex::DefaultSolver* ik_solver2;
};

} // end namespace volga_core

#endif /* __VOLGA_CORE_KINEMATIC_MODEL_H__ */
