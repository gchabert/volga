/*************************************************************************************
 *
 * Copyright (c) 2020-2021, IRT Jules Verne.
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

#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <ros/package.h>
#include <ibex.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>

#include <volga_core/KDLRobot.h>
#include <volga_core/LocalPlanner.h>
#include <volga_core/SerialRobot.h>
#include <volga_core/tf2ibex.h>

using namespace std;
using namespace ibex;
using namespace volga_core;

#define _PACKAGE_NAME "volga_examples"

int main(int argc, char** argv) {

	ros::init(argc, argv, "ik_test");

	std::string urdf_path = ros::package::getPath(_PACKAGE_NAME);
	//std::string urdf_filename = urdf_path + "/urdf/robot3p3r.urdf";
	std::string urdf_filename = urdf_path + "/urdf/iiwa-6joints.urdf";

	bool use_quaternion = true;
	bool log = true;
	volga_core::SerialRobot robot(urdf_filename, 0.1, false, use_quaternion, log);

	Vector _q(robot.fk.nb_actuated,0.12);

	IntervalVector xgoal = robot.fk(robot.fk.tool_name).eval_vector(_q);

	const CovSolverData& data=robot.ik(xgoal);

	cout << "found " << data.nb_solution() << " solutions." << endl << endl;
	for (int i =0; i<data.nb_solution(); i++) {
		cout << " solution n°" << (i+1) << ": " << data.solution(i) << endl << endl;
	}

}
