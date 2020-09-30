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

#include <tf_conversions/tf_kdl.h>
#include <volga_core/KDLRobot.h>

using namespace std;
using namespace ibex;

namespace volga_core {

void KDLRobot::init(const urdf::ModelInterface& model, const std::string& base_name, const std::string& tool_name, unsigned int n) {

	cout << "Building KDL model (optionnal)..." << endl;

	this->base_name = base_name;
	this->tool_name = tool_name;
	this->n = n;

	if (!kdl_parser::treeFromUrdfModel(model, my_tree)) {
		ROS_ERROR("Failed to construct kdl tree");
		exit(0);
	}

	fk_solver_ = new KDL::TreeFkSolverPos_recursive(my_tree);

	std::vector<std::string> endpoints;
	endpoints.push_back(tool_name);

	jac_solver_ = new KDL::TreeJntToJacSolver(my_tree);

	KDL::Chain chain;
	if (!my_tree.getChain(base_name, tool_name, chain)) {
		ROS_ERROR("Cannot extract base-to-tool chain from URDF");
		exit(1);
	}

	KDL::JntArray qmin(n);
	KDL::JntArray qmax(n);

	for (int i=0; i<n; i++) {
		//string joint_name = chain.getSegment(i).getJoint().getName();
		qmin(i)  = -1000; // todo
		qmax(i) = 1000;
	}

	tracik_solver = new TRAC_IK::TRAC_IK(chain, qmin, qmax, 1); // no real time: we allow 1s

	cout << "Done." << endl << endl;
}


KDLRobot::~KDLRobot() {
	delete fk_solver_;
	delete jac_solver_;
	delete tracik_solver;
}

tf::Transform KDLRobot::forward(const string& link_name, const Vector& q) const {
	KDL::Frame p_out;
	KDL::JntArray jq(n);

	for (unsigned int i=0; i<n; i++) jq(i)=q[i];

	// get pose of the effector
	if (fk_solver_->JntToCart(jq, p_out, link_name)!=0)
		throw std::runtime_error("FK solver returned with an error.");

	tf::Matrix3x3 m;
	for (unsigned int i=0; i<3; i++) {
		for (unsigned int j=0; j<3; j++)  {
			m[i][j]=p_out.M.data[i*3+j];
		}
	}

	tf::Vector3 v;
	for (unsigned int i=0; i<3; i++)
		v[i]=p_out.p.data[i];

	tf::Transform t;
	t.setOrigin(v);
	t.setBasis(m);

	return t;
}

Vector KDLRobot::inverse(const Vector& q, const tf::Transform& xgoal) const {
	KDL::JntArray q_init(n);
	KDL::JntArray q_out(n);
	KDL::Frames p_in;
	KDL::Frame  tool_frame;
	tf::transformTFToKDL(xgoal, tool_frame);
	p_in[tool_name]=tool_frame;

	for (unsigned int i=0; i<n; i++) {
		q_init(i)=q[i];
	}

	// NOTE: KDL solver does not work fine
	//	if (ik_solver_->CartToJnt(q_init, p_in, q_out)<0) {
	// --> TRAC_IK
	if (tracik_solver->CartToJnt(q_init, tool_frame, q_out)<0) {
		ROS_ERROR("Inverse kinematic failed.");
		exit(1);
	}

	Vector qgoal(n);
	for (unsigned int i=0; i<n; i++) {
		qgoal[i]=q_out(i);
	}

	return qgoal;
}

} /* namespace volga_core */
