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

#ifndef __VOLGA_CORE_SERIAL_ROBOT_H__
#define __VOLGA_CORE_SERIAL_ROBOT_H__

#include <ibex.h>

#include <tf/LinearMath/Transform.h>

#include "FKInterpol.h"
#include "Frame.h"
#include "KDLRobot.h"
#include "KinematicModel.h"
#include "Mesh.h"

namespace volga_core {

/**
 * \brief Ibex model of a serial Robot
 *
 */
class SerialRobot {
public:

	/**
	 * \brief Build the model of a serial robot.
	 *
	 * \param urdf            - The URDF description of the robot, which can either be:
	 *                          * the name of the robot parameter on the parameter server
	 *                            (if param==true)
	 *                          * or the full path of an URDF file
	 *                            (param==false)
	 * \param param           - Whether urdf is the name of a parameter (true) or a file (false).
	 * \param eps             - The precision required for collision checking. A distance
	 *                          between objects below eps will be considered as a collision.
	 * \param use_quaternions - Do we use a quaternion or rotation matrix?
	 * \param log             - Verbose if true
	 */
	SerialRobot(const std::string& urdf, double eps, bool param=true, bool use_quaternions=false, bool log=false);

	/**
	 * \brief Build the model of a serial robot.
	 *
	 * \param urdf            - The URDF model of the robot
	 * \param eps             - The precision required for collision checking. A distance
	 *                          between objects below eps will be considered as a collision.
	 * \param use_quaternions - Do we use a quaternion or rotation matrix?
	 * \param log             - Verbose if true
	 */
	SerialRobot(const urdf::ModelInterface& urdf, double eps, bool use_quaternions=false, bool log=false);

	/**
	 * \brief Delete this.
	 */
	~SerialRobot();

	/**
	 * \brief Interlpolation model function for a link.
	 */
	FKInterpol& fk_interp(uint link);

	/**
	 * \brief Get the mesh of the ith link.
	 *
	 * The order of links is that of the URDF model.
	 */
	Mesh& mesh(uint link_number);

	/*
	 * \brief Get the number of a link
	 */
	unsigned int link(const std::string& name);

	/*
	 * \brief True if the mesh of a link has a collision geometry.
	 */
	bool has_collision(uint link) const;

	/*
	 * \brief Name of the robot
	 */
	std::string get_name() const;

	/**
	 * Inverse kinematics
	 *
	 * \param xgoal - desired effector pose (see forward
	 *         		  kinematic model).
	 *
	 * \return data - the Ibex solver structure containing
	 *                all solutions
	 */
	const ibex::CovSolverData& ik(const ibex::IntervalVector& xgoal);

	/**
	 * \brief Forward kinematic model.
	 */
	KinematicModel fk;

	/*
	 * \brief For debug
	 */
	KDLRobot kdl;

	/**
	 * \brief Collision threshold.
	 *
	 * The precision required for collision checking. A distance
	 * between objects below epsilon will be considered as a collision.
	 */
	const double epsilon;

private:
	/*
	 * Initialize the model.
	 */
	void init(const urdf::ModelInterface& urdf, double eps, bool use_quaternions, bool log);

	/*
	 * Name of the robot (as in the urdf)
	 */
	std::string name;

	/*
	 * Trajectory of each link frame with linear interpolation of joint positions
	 */
	std::vector<FKInterpol*> _fk_interp;

	/*
	 * Map giving the number of each link.
	 */
	ibex::SymbolMap<unsigned int> link_numbers;

	/**
	 * Meshes of all links
	 *
	 * The order of links is that of the URDF model.
	 */
	std::vector<Mesh> link_meshes;

	/*
 	 * Whether the ith link has a collision geometry.
	 */
	bool* _has_collision;

};

} // end namespace volga_core

#endif /* __VOLGA_CORE_SERIAL_ROBOT_H__ */
