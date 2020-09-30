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

#ifndef __VOLGA_CORE_FK_INTERPOL_H__
#define __VOLGA_CORE_FK_INTERPOL_H__

#include <ibex.h>

#include <tf/LinearMath/Transform.h>

#include "Frame.h"
#include "KDLRobot.h"
#include "KinematicModel.h"
#include "Mesh.h"

namespace volga_core {

/**
 * \brief Interpolating function for the forward kinematic model of a link.
 *
 * Useful for local planning in joint space.
 *
 * This class also contains the Ibex system (constraints)
 * representing the collision of two balls, one in the
 * frame of the link and the other one fixed.
 *
 * The class resorts to mutable constants (references)
 * so that they can be dynamically updated.
 * This way, a single instance of the function/system is
 * created (=> static).
 */
class FKInterpol {
public:

	/**
	 * \brief Build the interpolation
	 *
	 * \param fk  - Forward model of a link
	 * \param log - verbose if true
	 */
	FKInterpol(const ibex::Function& fk, bool log=false);

	/**
	 * \brief Delete this.
	 */
	~FKInterpol();

	/**
	 * \brief Forward kinematic composed with joint interpolation.
	 *
	 * Return t -> fk((1-t)q1 + tq2)
	 */
	ibex::Function& get();

	/**
	 * \brief Check collision at a given time.
	 */
	bool check_collision(double t) const;

	/**
	 * \brief Get the solver for all collisions
	 */
	ibex::Solver& get_collision_solver();

	/**
	 * \brief Set starting joint position
	 */
	void set_q0(const ibex::Vector& q0);

	/**
	 * \brief Set ending joint position
	 */
	void set_q1(const ibex::Vector& q1);

	/**
	 * \brief Ball of the current node in the mesh of the robot link.
	 *
	 * Allows to dynamically set the right
	 * distance in the system to be solved.
	 */
	void set_link_ball(const Ball& ball);

	/**
	 * \brief Ball of the current mesh node in the mesh of the obstacle link.
	 *
	 * Allows to dynamically set the right
	 * distance in the system to be solved.
	 */
	void set_fixed_ball(const Ball& ball);

	/**
	 * \brief Forward kinematic model.
	 */
	KinematicModel fk;

private:

	/*
	 * Domain of the starting joint position (for fk_interpol)
	 */
	ibex::Domain* dq0;

	/*
	 * Domain of the ending joint position (for fk_interpol)
	 */
	ibex::Domain* dq1;

	/*
	 * Domain of the link ball origin in the frame (thin)
	 */
	ibex::Domain dlink_ball_origin;

	/*
	 * Domain of the link ball radius
	 */
	ibex::Domain dlink_ball_radius;

	/*
	 * Domain of the fixed ball origin in the world frame (thin)
	 */
	ibex::Domain dfixed_ball_origin;

	/*
	 * Domain of the fixed ball radius
	 */
	ibex::Domain dfixed_ball_radius;

	/*
	 * Trajectory of the frame with linear interpolation of joint positions
	 */
	ibex::Function fk_lerp;

	/*
	 * System that constraints a ball attached to the
	 * link frame and moving with linear interpolation
	 * of joint positions, to be in collision with
	 * a fixed ball (attached to the world frame).
	 */
	ibex::System* fk_collision;

	/*
	 * Solver for all collisions
	 * (tangency points).
	 */
	ibex::DefaultSolver* fk_solver;

};

} // end namespace volga_core

#endif /* __VOLGA_CORE_FK_INTERPOL_H__ */
