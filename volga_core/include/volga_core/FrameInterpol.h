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

#ifndef __VOLGA_CORE_FRAME_INTERPOL_H__
#define __VOLGA_CORE_FRAME_INTERPOL_H__


#include <ibex_System.h>
#include <ibex_DefaultSolver.h>
#include <ibex_TemplateDomain.h>
#include <tf/LinearMath/Transform.h>

#include "Ball.h"
#include "Point3D.h"
#include "UnitQuaternion.h"

namespace volga_core {

/**
* \brief Interpolating function for a frame in SO(3).
 *
 * Useful for local planning in Cartesian space.
 *
 * This class also contains the Ibex system (constraints)
 * representing the collision of two balls, one in the
 * interpolated frame and the other one fixed.

 * The class uses both spherical and linear interpolation
 * for the rotation, depending on initial/final frames.
 *
 * The class resorts to mutable constants (references)
 * so that they can be dynamically updated.
 * This way, a single instance of the function/system is
 * created (=> static).
 */
class FrameInterpol {
public:

	/**
	 * \brief Set starting and ending poses (or transformation)
	 */
	static void set_t0_t1(const tf::Transform& t0, const tf::Transform& t1);

	/**
	 * \brief Ball of the current node in the mesh attached to the frame.
	 *
	 * Allows to dynamically set the right
	 * distance in the system to be solved.
	 */
	static void set_link_ball(const Ball& ball);

	/**
	 * \brief Ball of the current mesh node in the mesh of the obstacle link.
	 *
	 * Allows to dynamically set the right
	 * distance in the system to be solved.
	 */
	static void set_fixed_ball(const Ball& ball);

	/**
	 * \brief Update the internal constants and return
	 * the appropriate interpolation function.
	 */
	static ibex::Function& interpol();

	/**
	 * \brief Check collision at a given time.
	 */
	static bool check_collision(double t);

	/**
	 * \brief Get the solver for all collisions (tangency points).
	 */
	static ibex::Solver& get_collision_solver();

private:

	/*
	 * Constructor for the singleton
	 */
	FrameInterpol();

	/*
	 * Delete the singleton
	 */
	~FrameInterpol();

	/*
	 * Set starting and ending poses (non-static variant)
	 */
	void _set_t0_t1(const tf::Transform& t0, const tf::Transform& t1);

	/*
	 * Get the singleton
	 */
	static FrameInterpol& get();

	/*
	 * Domain of the starting frame position (thin)
	 */
	ibex::Domain dx0;

	/*
	 * Domain of the ending frame position (thin)
	 */
	ibex::Domain dx1;

	/*
	 * Domain of the starting frame orientation: a (thin) validated unit quaternion
	 */
	ibex::Domain dq0x;
	ibex::Domain dq0y;
	ibex::Domain dq0z;
	ibex::Domain dq0w;

	/*
	 * Domain of the ending frame orientation: a (thin) validated unit quaternion
	 */
	ibex::Domain dq1x;
	ibex::Domain dq1y;
	ibex::Domain dq1z;
	ibex::Domain dq1w;

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
	 *  Dot product of q0 and q1.
	 *
	 *  Depending if the q0.q1~0 or q0.q1>>0, a different formula is used
	 *  for interpolation.
	 *
	 *  Used as an input constant as this node in the expression
	 *  does not depend on "t".
	 */
	ibex::Domain dqdot;

	/*
	 *  Angle between q0 and q1.
	 *
	 * Used as an input constant to avoid the introduction of acos operator in
	 * the slerp expression (this node does not depend on "t").
	 */
	ibex::Domain dtheta_0;

	/**
	 * Whether we use linear or spherical interpolation
	 * (set after calling set_t0_t1)
	 */
	bool is_lerp;

	/*
	 * Trajectory of the reference frame
	 * with quaternion linear interpolation + normalization
	 */
	ibex::Function lerp;

	/**
	 * Trajectory of the reference frame
	 * with quaternion spherical interpolation
	 * (along greatest circle)
	 */
	ibex::Function slerp;

	/*
	 * System with linear interpolation
	 */
	ibex::System* lerp_collision;

	/*
	 * System with spherical interpolation
	 */
	ibex::System* slerp_collision;

	/*
	 * Collision solver with linear interpolation
	 */
	ibex::DefaultSolver* lerp_solver;

	/*
	 * Collision solver with spherical interpolation
	 */
	ibex::DefaultSolver* slerp_solver;

	/*
	 * Build the quaternion structure of q0 from ibex domains
	 */
	UnitQuaternion build_q0();

	/*
	 * Build the quaternion structure of q1 from ibex domains
	 */
	UnitQuaternion build_q1();

	/*
	 * Build a simple linear interpolation of [x0,x1]
	 * for the translation.
	 */
	const ibex::ExprNode& build_x(const ibex::ExprSymbol& t);
};

} /* namespace volga_core */

#endif /* __VOLGA_CORE_FRAME_INTERPOL_H__ */
