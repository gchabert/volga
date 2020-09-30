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

#include <ibex_Exception.h>

// for comparison purpose
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <volga_core/KDLRobot.h>
#include <volga_core/tf2ibex.h>
#include <volga_core/KinematicModel.h>

using namespace std;
using namespace ibex;

namespace volga_core {

const ExprNode& get_vector(const urdf::Vector3& axis, const ExprNode& qi) {
	return ExprVector::new_col(axis.x*qi, axis.y*qi, axis.z*qi);
}


namespace {
string joint_type_name(const urdf::Joint& j) {
	switch (j.type) {
	case urdf::Joint::REVOLUTE:   return "revolute";
	case urdf::Joint::CONTINUOUS: return "continuous";
	case urdf::Joint::PRISMATIC:  return "prismatic";
	case urdf::Joint::FLOATING :  return "floating";
	case urdf::Joint::PLANAR:     return "planar";
	case urdf::Joint::FIXED:      return "fixed";
	case urdf::Joint::UNKNOWN :
	default :        return "unknown";
	}
}
}

KinematicModel::KinematicModel() : nb_actuated(0 /* tmp */), nb_links(0 /* tmp */),
		fk_links(NULL /* tmp */), joint_limits(1 /* tmp */), dx(Dim::col_vec(7)),
		ik_sys1(NULL), ik_solver1(NULL), ik_sys2(NULL), ik_solver2(NULL) {
}

void KinematicModel::init(const urdf::ModelInterface& model, bool use_quaternions, bool log) {

	(string&) base_name = model.getRoot()->name;

	// frame expressions associated to each link
	SymbolMap<Frame> link_frames;

	// symbols associated to each actuated link
	const char* joint_symbol_prefix="q";

	// initial translation expression (null vector)
	const ExprConstant& root_position = ExprConstant::new_vector(Vector::zeros(3),false);

	// initial rotation expression (identity)
	Rotation root_rotation = Rotation::identity(use_quaternions);

	urdf::LinkConstSharedPtr root = model.getRoot();

	Frame root_frame(root_position,root_rotation);

	link_frames.insert_new(root->name.c_str(), root_frame);

	stack<urdf::JointConstSharedPtr> jointstack;

	//----------------------------------------------------------------------------------
	/*
	 * Selection of ancestors.
	 *
	 * in the current version of the code, only the ancestors
	 * of the tool are processed. Note: generation of all
	 * link expressions would require careful memory cleanup.
	 */
//	{
//		urdf::LinkConstSharedPtr link = tool;
//		while (link->parent_joint) {
//			urdf::JointSharedPtr joint=link->parent_joint;
//			jointstack.push(joint);
//			if (joint->type!=urdf::Joint::FIXED) ((unsigned int&) nb_actuated)++;
//			link = model.getLink(joint->parent_link_name);
//		}
//		if (link!=root) {
//			cerr << "root link is not an ancestor of \"" << tool_name << "\"" << endl;
//			exit(1);
//		}
//	}

	/*
	 * Process every link:
	 */
	{
		stack<urdf::JointConstSharedPtr> tmp_jointstack;

		urdf::LinkConstSharedPtr link = root;
		stack<urdf::LinkConstSharedPtr> linkstack;
		linkstack.push(root);
		while (!linkstack.empty()) {
			urdf::LinkConstSharedPtr link = linkstack.top();
			linkstack.pop();
			vector<urdf::JointSharedPtr> joints = link->child_joints;
			for (vector<urdf::JointSharedPtr>::const_iterator it=joints.begin(); it!=joints.end(); ++it) {
				urdf::JointSharedPtr joint=*it;
				if (joint->type!=urdf::Joint::FIXED) ((unsigned int&) nb_actuated)++;
				tmp_jointstack.push(joint);
				linkstack.push(model.getLink(joint->child_link_name));
			}
		}

		// reverse the stack
		while (!tmp_jointstack.empty()) {
			jointstack.push(tmp_jointstack.top());
			tmp_jointstack.pop();
		}
	}

	if (log) cout << "Building geometric model....\n";

	const ExprSymbol& q = ExprSymbol::new_(joint_symbol_prefix, Dim::col_vec(nb_actuated));

	if (log) cout << "  Vector variable " << q.name << " for " << nb_actuated << " actuated joints" <<endl;

	int i=0; // actuated joint count

	joint_limits.resize(nb_actuated);

	try {
		while (!jointstack.empty()) {
			urdf::JointConstSharedPtr joint = jointstack.top();
			jointstack.pop();
			if (log) cout << "  Joint: "  <<  	 joint->name << " (" << joint_type_name(*joint) << ")";

			const Frame& parent_frame = link_frames[joint->parent_link_name.c_str()];

			const urdf::Pose& pose=joint->parent_to_joint_origin_transform;
			Frame child_origin_frame(parent_frame.move(pose.position), parent_frame.rotation * pose.rotation);

			const ExprNode* position = &child_origin_frame.position; // by default
			Rotation rotation = child_origin_frame.rotation; // by default

			if (joint->type!=urdf::Joint::FIXED) {
				if (joint->limits)
					joint_limits[i]=Interval(joint->limits->lower, joint->limits->upper);
				else {
					assert(joint->type==urdf::Joint::CONTINUOUS);
					joint_limits[i]=Interval::zero() | Interval::TWO_PI;
				}
				if (log) cout << " in " << joint_limits[i] << endl;
				switch (joint->type) {
				case urdf::Joint::PRISMATIC:
					position = &(*position + rotation * get_vector(joint->axis, q[i]));
					break;
				case urdf::Joint::REVOLUTE:
				case urdf::Joint::CONTINUOUS:
					rotation = rotation * Rotation(joint->axis, q[i], use_quaternions);
					break;
				default:
					cerr << " not implemented joint:" << joint_type_name(*joint) << endl;
				}
				joint2name.push_back(joint->name);
				name2joint.insert_new(joint->name.c_str(), i);
				const Frame& parent_frame = link_frames[joint->parent_link_name.c_str()];
				i++;
			}
			cout << endl;
			Frame child_frame(*position, rotation);
			link_frames.insert_new(joint->child_link_name.c_str(), child_frame);
		}
	} catch(SyntaxError& e) {
		cout << e << endl;
	}

	assert(nb_actuated>0);

	if (log) cout << endl << "  Base name: \"" << base_name << "\"\n";

	tool_name = model.getJoint(joint_name(nb_actuated-1))->child_link_name;
	if (log) cout << "  Tool name: \"" << tool_name << "\"" << endl;

	(unsigned int&) nb_links = link_frames.size();
	if (log) cout << "  Number of links: " << nb_links << endl << endl;

	fk_links = new Function[nb_links];
	vector<const ExprNode*> garbage;

	int l=0; // count links
	for (std::map<string, urdf::LinkSharedPtr>::const_iterator it=model.links_.begin(); it!=model.links_.end(); ++it) {
		Array<const ExprSymbol> _q(ExprSymbol::new_(joint_symbol_prefix, Dim::col_vec(nb_actuated)));

		const Frame& link_frame=link_frames[it->second->name.c_str()];
		Frame _link_frame=link_frame.copy(q,_q);

		fk_links[l].init(_q, _link_frame.stack(),(string("fk_")+it->second->name).c_str());
		if (log) cout << "  Function " << fk_links[l].name << " built. (size=" << fk_links[l].expr().size << ")" << endl;
		name2link.insert_new(it->second->name.c_str(),l);
		link2name.push_back(it->second->name);
		garbage.push_back(&link_frame.position);
		link_frame.rotation.add_nodes(garbage);
		l++;
	}

	// cleanup
	cleanup(garbage, true);

	if (log) cout << "Done.\n\n";
}

const ibex::CovSolverData& KinematicModel::inverse(const ibex::IntervalVector& x) {

	dx.v() = x;

	// we drop a component of the quaternion
	// to have a well-constrained system.
	// We will choose either the one or the
	// second (the one with less magnitude gives
	// better numerically conditioning).
	if (x[0].mag() > x[1].mag()) {
		if (!ik_sys1) {
			SystemFactory fac;
			const Function& fk=(*this)(tool_name);
			Array<const ExprSymbol> qs(fk.args().size());
			varcopy(fk.args(),qs);
			const ExprSymbol& q=qs[0];
			fac.add_var(q);

			const ExprConstant& _x = ExprConstant::new_mutable(dx);
			DoubleIndex idx=DoubleIndex::subcol(Dim::col_vec(7),1,6,0);
			const ExprCtr& ctr(fk(q)[idx]=_x[idx]);
			fac.add_ctr(ctr);
			ik_sys1 = new System(fac);
			ik_solver1 = new DefaultSolver(*ik_sys1);
			cleanup(ctr.e,true);
			delete &ctr;
		}

		ik_solver1->solve(joint_limits);
		return ik_solver1->get_data();

	} else {
		if (!ik_sys2) {
			SystemFactory fac;
			const Function& fk=(*this)(tool_name);
			Array<const ExprSymbol> qs(fk.args().size());

			varcopy(fk.args(),qs);
			const ExprSymbol& q=qs[0];
			fac.add_var(q);

			const ExprConstant& _x = ExprConstant::new_mutable(dx);
			const ExprNode& fkq=fk(q);
			DoubleIndex idx=DoubleIndex::subcol(Dim::col_vec(7),2,6,0);

			const ExprCtr& ctr1(fkq[0]=_x[0]);
			fac.add_ctr(ctr1);
			const ExprCtr& ctr2(fkq[idx]=_x[idx]);
			fac.add_ctr(ctr2);

			ik_sys2 = new System(fac);
			ik_solver2 = new DefaultSolver(*ik_sys2);
			cleanup(Array<const ExprNode>(ctr1.e,ctr2.e),true);
			delete &ctr1;
			delete &ctr2;
		}

		ik_solver2->solve(joint_limits);
		return ik_solver2->get_data();
	}
}

const Function& KinematicModel::operator()(uint link) const {
	return fk_links[link];
}

const Function& KinematicModel::operator()(const std::string& link_name) const {
	return fk_links[link_number(link_name)];
}

KinematicModel::~KinematicModel() {
	delete[] fk_links;
	if (ik_sys1) {
		delete ik_solver1;
		delete ik_sys1;
	}
	if (ik_sys2) {
		delete ik_solver2;
		delete ik_sys2;
	}
}

const string& KinematicModel::link_name(uint link) const {
	return link2name[link];
}

uint KinematicModel::link_number(const std::string& link_name) const {
	return name2link[link_name.c_str()];
}

const std::string& KinematicModel::joint_name(uint joint) const {
	return joint2name[joint];
}

uint KinematicModel::joint_number(const std::string& joint_name) const {
	return name2joint[joint_name.c_str()];
}

tf::Transform KinematicModel::check_with_tf(uint joint, const Vector& q) const {
	return _tf_transform((*this)(joint).eval_matrix(q).mid());
}

} // end namespace volga_core
