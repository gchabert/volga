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

#include <ibex_DefaultSolver.h>

#include <algorithm>
#include <volga_core/LocalPlanner.h>
#include <volga_core/tf2ibex.h>

using namespace std;
using namespace ibex;

namespace volga_core {

LocalPlanner::LocalPlanner(Mesh& mesh, Mesh& obstacle, const tf::Transform& t1, const tf::Transform& t2) :
	log(false), update_mesh(false), link_mesh(mesh), fixed_mesh(obstacle), fk_interp(NULL), space(CARTESIAN) {

	FrameInterpol::set_t0_t1(t1, t2);
	solver = &FrameInterpol::get_collision_solver();

	// only for max_time_rec that only works for a
	// single ball as obstacle
	FrameInterpol::set_fixed_ball(obstacle.root->ball);
}

LocalPlanner::LocalPlanner(SerialRobot& robot, unsigned int link, SpaceType space, Mesh& obstacle, const Vector& q1, const Vector& q2) :
		log(false), update_mesh(false), link_mesh(robot.mesh(link)), fixed_mesh(obstacle), space(space) {

	if (space==CARTESIAN) {
		tf::Transform t1 = _tf_transform(robot.fk(link).eval_matrix(q1).mid());
		tf::Transform t2 = _tf_transform(robot.fk(link).eval_matrix(q2).mid());

		// OR:
		//	tf::Transform t1 = robot.kdl.forward(q0);
		//	tf::Transform t2 = robot.kdl.forward(q1);
		fk_interp = NULL;
		FrameInterpol::set_t0_t1(t1, t2);
		solver = &FrameInterpol::get_collision_solver();
		// only for max_time_rec that only works for a
		// single ball as obstacle
		FrameInterpol::set_fixed_ball(obstacle.root->ball);

	} else {
		fk_interp = &robot.fk_interp(link);
		fk_interp->set_q0(q1);
		fk_interp->set_q1(q2);
		solver = &fk_interp->get_collision_solver();
		// only for max_time_rec that only works for a
		// single ball as obstacle
		fk_interp->set_fixed_ball(obstacle.root->ball);
	}
}

double LocalPlanner::max_time(double eps_meters) {

	//double tmax=max_time_rec(solver, *mesh.root, 0, 1);   ----> deprecated
	double tmax=max_time2(eps_meters);

	if (update_mesh) {
		link_mesh.pops.sort([](const pair<double,long>& x, const pair<double,long>& y) { return x.first < y.first; } );
		fixed_mesh.pops.sort([](const pair<double,long>& x, const pair<double,long>& y) { return x.first < y.first; } );

		if (log) {
			cout << "link pops:" << endl;
			for (list<pair<double, long> >::const_iterator it=link_mesh.pops.begin(); it!=link_mesh.pops.end(); ++it) {
				cout << "  " << it->first << " : " << it->second << endl;
			}

			cout << endl << "obstacle pops:" << endl;
			for (list<pair<double, long> >::const_iterator it=fixed_mesh.pops.begin(); it!=fixed_mesh.pops.end(); ++it) {
				cout << "  " << it->first << " : " << it->second << endl;
			}

		}
	}
	return tmax;
}

LocalPlanner::Event::Event(double t, EventType type, Mesh::Node* node1, Mesh::Node* node2, const CovSolverData& data) :
		t(t), type(type), node1(node1), node2(node2), data(data,true) {

}

bool LocalPlanner::next_sol(Mesh::Node* node1, Mesh::Node* node2, const CovSolverData& data, const IntervalVector*& t) {
	CovSolverData::BoxStatus box_status;

	if (space==JOINT) {
		fk_interp->set_link_ball(node1->ball);
		fk_interp->set_fixed_ball(node2->ball);
	} else {
		FrameInterpol::set_link_ball(node1->ball);
		FrameInterpol::set_fixed_ball(node2->ball);
	}

	if (data.nb_pending()>0 && data.pending(0)[0].lb()==0.0) {
		// a terrible hack to check if there is a collision at initial time
		// without compromising the overall algorithm structure: we
		// add a "fake" solution at t=0 and remove 0 from the domain.
		if ((space==JOINT && fk_interp->check_collision(0)) ||
			(space==CARTESIAN && FrameInterpol::check_collision(0))) {
			CovSolverData data_without_0(1,1,0);
			IntervalVector zero(1,0.);
			data_without_0.add_solution(zero);
			data_without_0.add_pending(IntervalVector(1,Interval(ibex::next_float(0),data.pending(0)[0].ub())));
			solver->start(data_without_0);
			solver->flush();
			t = &solver->get_data().solution(0);
			return true;
		}
	}

	solver->start(data);

	bool res = solver->next(box_status, &t);
	solver->flush();

	if (res) {
		if (box_status!=CovSolverData::SOLUTION) {
			double eps = 1e-4;
			while (res && box_status!=CovSolverData::SOLUTION && eps>1e-12) {
				ROS_WARN_STREAM("Interval solver precision increased: eps=" << eps);
				(double&) solver->eps_x_max[0] = eps;
				solver->start(solver->get_data());
				res = solver->next(box_status, &t);
				solver->flush();
				eps *=0.1;
			}
			if (eps<=1e-12) {
				ROS_ERROR("Interval solver failed: abort");
				exit(1);
			}
		}
	}
	return res;
}

list<LocalPlanner::Event*>::iterator LocalPlanner::pos(list<LocalPlanner::Event*>::iterator start, double t) {
	// important note:
	// The instruction ++start and the condition e->t >= t are important
	// in the case an OPEN event occurs for a node at the same time of
	// its father node.
	// - If we start find_if from "start" or even the beginning of the list,
	//   the new event would be inserted *before* the current iterator (and the
	//   event would be skipped by the loop in max_time2).
	// - If we replace the condition e->t >= t by e->t > t to avoid this
	//   problem, this would lead to another problem: in the (frequent) case
	//   of both objects physically in collision at initial time (t=0),
	//   all the OPEN events will be added at the end of the list
	//   while traversing the mesh structure downto a leaf for each object.
	//   This means that the collision would be detected after a largest-first
	//   tree search in the case, which is very inefficient. Using e->t >= t leads
	//   to a depth-first search.
	++start;
	return find_if(start, timeline.end(), [&t](const Event* e) { return e->t >= t; });
}

double LocalPlanner::max_time2(double eps_meters) {

	bool collision = false;
	double tmax = 1; // by default

	const IntervalVector* t;
	CovSolverData new_data(1,1,0);
	new_data.add_pending(IntervalVector(1,Interval(0,1)));

	if (next_sol(link_mesh.root, fixed_mesh.root, new_data, t)) {
		timeline.push_back(new Event((*t)[0].lb(),OPEN,link_mesh.root,fixed_mesh.root,solver->get_data()));
	}

	list<Event*>::iterator it = timeline.begin();

	while (it!=timeline.end()) {
		Event& e=**it;
		if (e.type==OPEN) {
			if (log)
				cout << "t=" << e.t << " " << e.node1->num_id << " ^ " << e.node2->num_id << " open" << endl;

			bool closed_event;
			// t_ub is the upper bound for the solver. It is set by default to 1 (max)
			// or to the current pop closing time, if any.
			// This bound is important for sub-node events. When a pop is closed for
			// a sub-node, the search for a new pop of this sub-node should only be
			// made inside the enclosing node pop time window.
			double t_ub=1.0;
			// calculate first the time when this node
			// is "closed" (no more ball intersection)
			// (pursue solving)
			if ((closed_event = next_sol(e.node1, e.node2, e.data, t))) {
				t_ub = (*t)[0].ub();
				if (t_ub<=e.t) {
					ROS_ERROR_STREAM("t_ub=" << t_ub << " e.t=" << e.t << endl);
					exit(0);
				}
				assert(t_ub>e.t);
				timeline.insert(pos(it,t_ub), new Event(t_ub, CLOSE, e.node1, e.node2, solver->get_data()));
			}
			// else: nothing to do. The CLOSE event in the time line merely serves to know
			// at which point in time it is necessary to calculate the
			// other solutions to balls intersection. If the CLOSE event
			// exceeds t=1, no more intersection (hence, OPEN event)
			// is to be considered.

			// Note: the center of the smallest circle enclosing a
			// triangle is always inside the triangle so the distance between
			// a leaf ball and the triangle is <= radius <= eps_meters/2 (see Mesh::refine)
			// But for an arbitrary node, the distance between the node ball and the part of
			// the mesh it contains can tend to the diameter when the number of subnodes grows.
			// So a node with radius less than eps_meters/4 is considered as
			// a "leaf" so that for two intersecting "leaves" it is proved that
			// two mesh points are at most at distance eps_meters, i.e., collision

			// determine the 'candidate' ball to pop.
			// if one is a leaf: this is the other one,
			// else, the one of smallest level.
			bool pop_fixed;
			if (e.node1->leaf() || e.node1->ball.radius<=eps_meters/4.) pop_fixed = true;
			else if (e.node2->leaf() || e.node2->ball.radius<=eps_meters/4.) pop_fixed = false;
			//else if (e.node1->level > e.node2->level) pop_fixed = true;
			else if (e.node1->ball.radius < e.node2->ball.radius) pop_fixed = true;
			else pop_fixed = false;

			Mesh::Node& node = pop_fixed? *e.node2 : *e.node1;

			if (log)
				cout << "\tpop node from mesh " << (pop_fixed? "2" : "1") << " id=" << node.num_id << endl;


			if (node.leaf() || node.ball.radius<=eps_meters/4.) {
				// if both are leaves --> collision
				collision = true;
				// we create the collision event for both nodes to have
				// both nodes painted in red graphically (in collision).
				if (update_mesh) {
					e.node1->add_collision(e.t);
					e.node2->add_collision(e.t);
					link_mesh.pops.push_back(make_pair(e.t, e.node1->num_id));
					fixed_mesh.pops.push_back(make_pair(e.t, e.node2->num_id));
				}
				tmax = e.t;
				break;
			} else {
				// note: the node does not necessarily becomes FREE when it quits the
				// first intersection it enters: intersections may be interleaved.
				if (update_mesh) {
					if (node.add_intersection(e.t, t_ub)) { // first intersection = pop
						if (pop_fixed)
							fixed_mesh.pops.push_back(make_pair(e.t, node.num_id));
						else
							link_mesh.pops.push_back(make_pair(e.t, node.num_id));
					}
				}
			}

			// pop the candidate ball
			Mesh::Fusion& f=(Mesh::Fusion&) node;

			// we start a new search for each child
			// (limited to current pop time window)
			CovSolverData new_data(1,1,0);
			new_data.add_pending(IntervalVector(1,Interval(e.t,t_ub)));

			// place OPEN events for each child
			for (vector<Mesh::Node*>::iterator it2=f.children.begin(); it2!=f.children.end(); ++it2) {
				if (next_sol(pop_fixed? e.node1 : *it2, pop_fixed? *it2 : e.node2, new_data, t)) {
					if (log)
						cout << "\t\t  next pop with id=" << (*it2)->num_id << " at t=" <<  (*t)[0].lb() << endl;
					timeline.insert(pos(it,(*t)[0].lb()), new Event((*t)[0].lb(), OPEN, pop_fixed? e.node1 : *it2, pop_fixed? *it2 : e.node2, solver->get_data()));
				} // else: no more pops with this node (inside the enclosing node pop time window).
			}
		} else {
			if (log) cout << "t=" << e.t << " " << e.node1->num_id << " ^ " << e.node2->num_id << " closed" << endl;
			// Find the next intersection
			if (next_sol(e.node1, e.node2, e.data, t)) {
				timeline.insert(pos(it,(*t)[0].lb()), new Event((*t)[0].lb(), OPEN, e.node1, e.node2, solver->get_data()));
			} // else: no more intersection!
		}

		++it;
	}

	for (std::list<Event*>::iterator it=timeline.begin(); it!=timeline.end(); ++it)
		delete *it;

	timeline.clear();

	return tmax;
}

double LocalPlanner::max_time_rec(Mesh::Node& node, double tinf, double tsup) {

	// note: active_times may have already been filled by
	// previous calls (with earlier time span [tinf,tsup]

	// change mutable constants
	if (space==JOINT)
		fk_interp->set_link_ball(node.ball);
	else
		FrameInterpol::set_link_ball(node.ball);

	Solver::Status status = solver->solve(IntervalVector(1,Interval(tinf, tsup)));

	// we need to copy the solver data
	// because the same solver will be used recursively
	CovSolverData data(solver->get_data(), true); // true means copy

	//cout << "status " << status << endl;
	//cout << solver->get_data() << endl;
	int nb_sol=data.nb_solution();

	// index of the solution which represents a
	// guaranteed time just before the next collision
	int k = 0;

	string skip="";

	if (log) {
		for (int i=0; i<node.id.size(); i++) skip+=' ';
		cout << skip;
		for (int i=0; i<node.id.size(); i++) cout << '*';
		cout << node.id << " (" << node.num_id << ")" << endl;
	}

	while (k<nb_sol) {
		// time before next collision
		double t_collision = data.solution(k)[0].lb();
		// works until commit a6f18cf8a11012e5e6f2dea6a511be9ab5fa7eb:
		// node.event_times.push_back(make_pair(t_collision, Mesh::Node::POP));
		link_mesh.pops.push_back(make_pair(t_collision, node.num_id));

		if (log) {
			cout << skip << (node.leaf()? "obstacle" : "pop") << ": " << t_collision;
			if (k<nb_sol-1) cout << " --> "<< data.solution(k+1)[0].ub();
			cout << endl;
		}

		if (node.leaf()) {
			return t_collision;
		} else {

			// note: it is possible to have k=nb_sol even if tsup<1
			// because of the "optimization" below.
			k++;
			// time after next collision
			// (may exceeds tsup -> not found by solver)
			double t_free = k<nb_sol? data.solution(k)[0].ub() : tsup;

			Mesh::Fusion& f=(Mesh::Fusion&) node;

			vector<double> times_before_collision;
			using iterator=std::vector<Mesh::Node*>::iterator;

			double min_time_before_collision=t_free; // by default
			for (iterator it=f.children.begin(); it!=f.children.end(); ++it) {
				double t_child_before_collision=max_time_rec(**it, t_collision, min_time_before_collision);
				if (t_child_before_collision<min_time_before_collision) {
					min_time_before_collision = t_child_before_collision;
					// optimization: useless to go further -> set as time upper bound for next child.
				}
			}
			if (min_time_before_collision<t_free) {
				//node.active_times.push_back(make_pair(min_time_before_collision, Mesh::Node::OBSTACLE));
				if (log)
					cout << skip << "child obstacle:" << min_time_before_collision << endl;

				return min_time_before_collision;
			} else {
				if (log) {
					cout << skip << "free:" << t_free;
					if (k<nb_sol) cout << " --> "<< data.solution(k)[0].ub();
					cout << endl;
				}
				// we are only sure to be free if there is actually a k^th solution;
				// otherwise, we may just have benefited from tsup (imposed by brother nodes,
				// see the "optimization").
				// works until commit a6f18cf8a11012e5e6f2dea6a511be9ab5fa7eb:
				//if (k<nb_sol) node.event_times.push_back(make_pair(t_free, Mesh::Node::FREE));
				// the obstacle is passed. Try to pass next one until there is none.
				k++;
			}
		}
	}
	if (log)
		if (nb_sol==0) cout << skip << "(free)" << endl;

	// if we have not returned in the while loop, there is no collision.
	return tsup;
}


} /* namespace volga_core */

