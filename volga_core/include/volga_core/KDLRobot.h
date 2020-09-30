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

#ifndef __VOLGA_CORE_KDLROBOT_H__
#define __VOLGA_CORE_KDLROBOT_H__

#include <ibex_Vector.h>
#include <tf/LinearMath/Transform.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <trac_ik/trac_ik.hpp>

namespace volga_core {

/**
 * \brief KDL model (for debug)
 */
class KDLRobot {
public:
	/**
	 * \brief BUild the model.
	 */
	void init(const urdf::ModelInterface& model, const std::string& base_name, const std::string& tool_name, unsigned int n);

	/**
	 * \brief Forward kinematics
	 */
	tf::Transform forward(const std::string& link_name, const ibex::Vector& q) const;

	/**
	 * \brief Inverse kinematics
	 */
	ibex::Vector inverse(const ibex::Vector& qestim, const tf::Transform& xgoal) const;

	/**
	 * \brief Delete this.
	 */
	~KDLRobot();

	/*
	 * Number of joints
	 */
	unsigned int n;

	std::string base_name;
	std::string tool_name;

protected:
	KDL::Tree my_tree;
	KDL::TreeFkSolverPos_recursive *fk_solver_;
	KDL::TreeJntToJacSolver *jac_solver_;
	TRAC_IK::TRAC_IK *tracik_solver;

};
} /* namespace volga_core */

#endif /* __VOLGA_CORE_KDLROBOT_H__ */

