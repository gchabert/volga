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

#ifndef __VOLGA_CORE_LOCAL_PLANNER_H__
#define __VOLGA_CORE_LOCAL_PLANNER_H__

#include <ibex_System.h>
#include <ibex_DefaultSolver.h>
#include <ibex_Solver.h>
#include "FrameInterpol.h"
#include "KinematicModel.h"
#include "SerialRobot.h"

namespace volga_core {

/**
 * \brief Local planner.
 *
 * This class maximizes the displacement of a robot in a fixed direction (in either joint
 * or Cartesian space) such that no collision occurs.
 *
 * The robot and obstacle geometries are defined by triangle meshes.
 *
 */
class LocalPlanner {
public:

	/**
	 * \brief Space type.
	 *
	 * JOINT :    the planner maximizes the robot displacement
	 *            along a straight line in the joint space
	 * CARTESIAN: the planner maximizes displacement along
	 *            interpolation in SO(3)
	 */
	typedef enum { JOINT, CARTESIAN } SpaceType;

	/**
	 * \brief Build the local planner
	 *
	 * The local planner will find the maximal displacement of the mesh
	 * (in argument) without collision along a given curve in SO(3).
	 *
	 * Computes the maximal t in [0,1] such that no collision
	 * occurs when the mesh pose is x(t) interpolating x0 and x1.
	 *
	 * \param link     - object (collision) geometry
	 * \param obstacle - obstacle geometry (single ball so far)
	 * \param x0       - initial pose of the object
	 * \param x1       - final pose of the object
	 */
	LocalPlanner(Mesh& link, Mesh& obstacle, const tf::Transform& x0, const tf::Transform& x1);

	/**
	 * \brief Build the local planner
	 *
	 * The local planner will find the maximal movement of the robot such that the
	 * link (given in argument) is collision-free with the obstacle (given in argument).
	 *
	 * Computes the maximal t in [0,1] such that no collision
	 * occurs when the robot configuration is q(t).
	 *
	 * \param robot    - the robot
	 * \param link     - link number
	 * \param space    - JOINT or CARTESIAN, see #SpaceType.
	 * \param obstacle - obstacle geometry (single ball so far)
	 * \param q0       - initial configuration of the robot
	 * \param q1       - final configuration of the robot
	 */
	LocalPlanner(SerialRobot& robot, unsigned int link, SpaceType space, Mesh& obstacle, const ibex::Vector& q0, const ibex::Vector& q1);

	/**
	 * \brief Return the maximal time before collision.
	 *
	 * \param eps_meters : distance below which we consider
	 *                     that a collision occurs.
	 */
	double max_time(double eps_meters);

	/**
	 * \brief Verbose if set to true.
	 */
	bool log;

	/**
	 * \brief Whether the mesh structure is updated with
	 * events
	 *
	 * This is useful for displaying bubbles under RVIZ
	 * but it takes time. Should be set to false most
	 * of the time.
	 */
	bool update_mesh;

protected:
	/*
	 * Type of events
	 */
	typedef enum { OPEN, CLOSE } EventType;

	/*
	 * An event is when two balls either enter or
	 * quit collision while the robot is moving.
	 */
	class Event {
	public:
		/*
		 * Build an event
		 * \param t     - the time of the event in [0,1]
		 * \param node1 - node in the robot mesh
		 * \param node2 - node in the obstacle mesh
		 * \param data  - current state of the solver that calculates all the
		 *                tangentially between these two nodes (allows to
		 *                iterate between solutions).
		 */
		Event(double t, EventType, Mesh::Node* node1, Mesh::Node* node2, const ibex::CovSolverData& data);
		/*
		 * Time of the event
		 */
		double t;

		/*
		 * Type of event
		 */
		EventType type;

		/*
		 * First node
		 */
		Mesh::Node* node1;

		/*
		 * Second node
		 */
		Mesh::Node* node2;

		/*
		 * Solver state
		 */
		ibex::CovSolverData data;
	};

	/*
	 * Compute the max time.
	 */
	double max_time2(double eps_meters);

	/*
	 * Compute the max time (deprecated)
	 */
	double max_time_rec(Mesh::Node& node, double tinf, double tsup);

	/*
	 * Return the position in the list where an event
	 * at time t should be placed.
	 * Allow to insert the new event in the time line.
	 */
	std::list<LocalPlanner::Event*>::iterator pos(std::list<LocalPlanner::Event*>::iterator start, double t);

	/*
	 * Return the next solution of the solver
	 * for two given nodes.
	 */
	bool next_sol(Mesh::Node* node1, Mesh::Node* node2, const ibex::CovSolverData& data, const ibex::IntervalVector*& t);

	/*
	 * Mesh of the current moving object (e.g., robot link)
	 * with which we check collision.
	 */
	Mesh& link_mesh;

	/*
	 * Mesh of the obstacle
	 */
	Mesh& fixed_mesh;

	/**
	 * Solver for collisions
	 */
	ibex::Solver* solver;

	/*
	 * Interpolation function for the current link.
	 * NULL if none (i.e., pure 6dof planning)
	 * Only used if planning works in joint space.
	 */
	FKInterpol* fk_interp;

	/*
	 * Whether we make interpolation in joint
	 * or Cartesian space.
	 */
	SpaceType space;

	/*
	 * Main structure storing all events.
	 */
	std::list<Event*> timeline;

};

} /* namespace volga_core */

#endif /* __VOLGA_CORE_LOCAL_PLANNER_H__ */
