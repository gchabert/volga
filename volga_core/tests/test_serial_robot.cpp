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

#include <volga_core/SerialRobot.h>
#include <volga_core/tf2ibex.h>
#include <ros/package.h>
#include <list>

using namespace std;
using namespace volga_core;
using namespace ibex;

#define _PACKAGE_NAME "volga_core"

int main() {
	bool use_quaternions = false;

	string urdf_path = ros::package::getPath(_PACKAGE_NAME);
	string urdf_filename = urdf_path + "/tests/robot3p3r.urdf";
	volga_core::SerialRobot robot(urdf_filename, 0.1, false, use_quaternions, true);

	int n=robot.fk.nb_actuated;

	Vector q(n);
	for (unsigned int i=0; i<n; i++) {
		q[i]=0.1; // just to give some "random" value
	}

	cout << "****** Checking with KDL ******" << endl;
	for (uint i=0; i<robot.fk.nb_links; i++) {
		string link_name=robot.fk.link_name(i);
		cout << "Link: " <<  link_name << endl;

		cout << "  KDL finds:  ";
		cout << "   " << robot.kdl.forward(link_name,q);
		cout << endl;

		cout << "  IBEX finds: \n";
		Matrix F = robot.fk(i).eval_matrix(q).mid();
		cout << "   origin=" << F.col(3) << endl << endl;
		cout << "   R=" << F.submatrix(0,2,0,2) << endl << endl;
	}


	// **** Test interpolation *****
	Vector q0(n,0.1);
	Vector q1(n,0.2);
	for (int i=0; i<robot.fk.nb_links; i++) {
		cout << "Link: " << robot.fk.link_name(i) << endl;
		cout << "  KDL finds:  ";
		cout << "   " << robot.kdl.forward(robot.fk.link_name(i),0.5*q0+0.5*q1);
		cout << endl;

		cout << "  IBEX finds: \n";
		robot.fk_interp(i).set_q0(q0);
		robot.fk_interp(i).set_q1(q1);
		cout << '\t' << _tf_transform(robot.fk_interp(i).get().eval_matrix(IntervalVector(1,0.5)).mid()) << endl;
	}
}
