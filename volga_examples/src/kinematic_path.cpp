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
#include <volga_core/tf2ibex.h>
#include <volga_core/KinematicModel.h>

using namespace std;
using namespace ibex;

constexpr double RATE = 30;

#define _PACKAGE_NAME "volga_examples"

std::ostream& operator<<(std::ostream& os, const sensor_msgs::JointState::_position_type& q) {
	os << '[';
	for (int i = 0; i < q.size(); i++) {
	  os << q[i];
	  if (i<q.size()-1) os << ' ';
	}
	os << ']';
	return os;
}

sensor_msgs::JointState to_joint_state_ur10(const Vector& _q) {
	sensor_msgs::JointState q;
	q.position.resize(6);
	q.name.resize(6);

	for (int i=0; i<6; i++)
		q.position[i]=_q[i];

	q.name[0]="shoulder_pan_joint";
	q.name[1]="shoulder_lift_joint";
	q.name[2]="elbow_joint";
	q.name[3]="wrist_1_joint";
	q.name[4]="wrist_2_joint";
	q.name[5]="wrist_3_joint";

	return q;
}

sensor_msgs::JointState to_joint_state_panda(const Vector& _q) {
	sensor_msgs::JointState q;
	q.position.resize(9);
	q.name.resize(9);

	for (int i=0; i<9; i++)
		q.position[i]=_q[i];

	q.name[0]="panda_joint1";
	q.name[1]="panda_joint2";
	q.name[2]="panda_joint3";
	q.name[3]="panda_joint4";
	q.name[4]="panda_joint5";
	q.name[5]="panda_joint6";
	q.name[6]="panda_joint7";
	q.name[7]="panda_finger_joint1";
	q.name[8]="panda_finger_joint2";

	return q;
}

Vector q0_panda() {
	double _q0[] = {0.0, -0.7849999999999999, 0.0, -2.356, 0.0, 1.5710000000000002, 0.785, 0.035, 0.035 };
	return Vector(9, _q0);
}

Vector q1_panda() {
	double _q1[] = {0.83567004314, 0.0985335081901,-1.42177066459,-1.23391835395,0.102529930748,1.25022099306,0.231728376746, 0.035, 0.035 };
	return Vector(9,_q1);
}

Vector q0_ur10() {
	return Vector(6, 0.1);
}

Vector q1_ur10() {
	double _q1[] = {-1.25789, -1.31569, -1.58587, 4.36431, 4.38567, -3.20316 };
	return Vector(6,_q1);
}

using namespace volga_core;

// usage: rosrun volga_core kinematic_path [link_name]"

int main(int argc, char** argv) {

	ros::init(argc, argv, "kinematic_path");

	ros::NodeHandle node;

	string urdf_param = "/robot_description";

	if (argc<3) {
		ROS_ERROR("\"obstacle\" and \"space\" arguments required");
		exit(0);
	}

	string obstacle_name = argv[1];
	if (obstacle_name!="fork" && obstacle_name!="cube") {
		ROS_ERROR_STREAM("unknown obstacle \"" << obstacle_name << "\"" << endl);
		exit(0);
	}

	string space_name = argv[2];
	if (space_name!="joint" && space_name!="cartesian") {
		ROS_ERROR_STREAM("invalid space name \"" << space_name << "\"" << endl);
		exit(0);
	}

	double epsilon = 0.03; // collision threshold

	bool use_quaternions = true;

	volga_core::SerialRobot robot(urdf_param, epsilon, true, use_quaternions, true);

	cout << "Robot:" << robot.get_name() << endl;

	string world_name=robot.fk.base_name;
	string tool_name =robot.fk.tool_name;
	uint tool=robot.fk.link_number(tool_name);

	uint n=robot.fk.nb_actuated;

	Vector q0=robot.get_name()=="panda" ? q0_panda() : q0_ur10();
	Vector q1=robot.get_name()=="panda" ? q1_panda() : q1_ur10();

	// For interpolation in Cartesian space
	tf::Transform x0=use_quaternions ?
			_tf_transform(robot.fk(tool_name).eval_vector(q0).lb()) :
			_tf_transform(robot.fk(tool_name).eval_matrix(q0).lb());

	tf::Transform x1=use_quaternions ?
			_tf_transform(robot.fk(tool_name).eval_vector(q1).lb()) :
			_tf_transform(robot.fk(tool_name).eval_matrix(q1).lb());

	LocalPlanner::SpaceType space = space_name=="joint"? LocalPlanner::JOINT : LocalPlanner::CARTESIAN;

	// place an obstacle in the middle of the trajectory of the tool
	// with a shift of 10 cm in z axis
	robot.fk_interp(tool).set_q0(q0);
	robot.fk_interp(tool).set_q1(q1);
	Function& interp_joint_obs=robot.fk_interp(tool).get();

	// 0.12 : pass
	// 0.13 : collision
	Point3D x_obs;

	if (space==LocalPlanner::JOINT)
		if (use_quaternions)
			x_obs = Point3D(interp_joint_obs.eval_vector(IntervalVector(1,Interval(0.5))).subvector(4,6).mid());
		else
			x_obs = Point3D(interp_joint_obs.eval_matrix(IntervalVector(1,Interval(0.5))).col(3).mid());
	else
		x_obs = _ibex(x0.getOrigin().lerp(x1.getOrigin(),0.5));

	Mesh obstacle;
//	obstacle.init(Ball(x_obs, 0.05));
	string volga_path = ros::package::getPath(_PACKAGE_NAME);

	// ===================== a big cube as obstacle =========================
	double vertices[8*3] = {
	0.5,0.5,-0.5
	,0.5,-0.499999,-0.5
	,-0.499999,-0.499999,-0.5
	,-0.499999,0.5,-0.5
	,0.5,0.5,0.5
	,-0.499999,0.5,0.5
	,-0.499999,-0.499999,0.5
	,0.5,-0.5,0.5 };

	unsigned int triangles[12*3] = {
	 0,1,2
	,2,3,0
	,4,5,6
	,6,7,4
	,0,4,7
	,7,1,0
	,1,7,6
	,6,2,1
	,2,6,5
	,5,3,2
	,4,0,3
	,3,5 };

//	obstacle.init(8,vertices,12,triangles,true,0.1,true);

	if (obstacle_name=="cube") {
		string obstacle_STL_filename = volga_path + "/meshes/cube.stl";
		obstacle.init(obstacle_STL_filename, true, epsilon, true);
		obstacle.move(tf::Transform(tf::Matrix3x3::getIdentity(),tf::Vector3(2 /*0.910001*/,0,0.03)));
	} else {
		string obstacle_STL_filename = volga_path + "/meshes/fork_light.stl";
		obstacle.init(obstacle_STL_filename, false, epsilon, true);
		obstacle.move(tf::Transform(tf::Matrix3x3::getIdentity(),_tf(x_obs)));
	}

	int marker_id=1;

	uint link = robot.link(tool_name);

	//visualization_msgs::Marker mesh_marker = robot.mesh(tool).markers_STL(marker_id++, tool_name, 1/RATE);
	visualization_msgs::Marker obs_marker = obstacle.markers_facets(marker_id++, world_name, 1/RATE);
	//visualization_msgs::Marker obs_marker = obstacle.markers_STL(marker_id++, world_name, 1/RATE);
	//visualization_msgs::Marker obs_marker = obstacle.marker_ball(marker_id++, 0, world_name, 1/RATE);

	// Calculate maximal time until collision, for each link
	cout << "Running planner..." << endl;
	double topt = 1;

	uint link_hit = tool; // by default

	for (int i=0; i<robot.fk.nb_links; i++) {
		if (!robot.has_collision(i)) continue;
		LocalPlanner planner(robot,i,space,obstacle,q0,q1);
		//planner.log = true;
		double topt_i = planner.max_time(epsilon);
		cout << "  max time for link " << i << ": " << topt_i << endl;
		if (topt_i<topt) { topt=topt_i; link_hit = i; }
	}

	// recompute to update the mesh structure (in particular, obstacle)
	cout << "recompute to update mesh information (for graphics)...\n";
	robot.mesh(link_hit).cleanup();
	LocalPlanner planner(robot,link_hit,space,obstacle,q0,q1);
	planner.update_mesh = true;
	planner.max_time(epsilon);


	cout << "Done!" << endl;
	cout << "  tmax=" << topt << endl;
	if (topt<1)
		cout << "  link first in collision : \"" << robot.fk.link_name(link_hit) << "\"" << endl;
	else
		cout << "  no link in collision!" << endl;

	// publish pose of the obstacles
	ros::Publisher obs_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// publish pose of the original mesh
	ros::Publisher mesh_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// publish pose of the active bubbles of the robot
	ros::Publisher bubbles1_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

	// publish pose of the active bubbles of the obstacle
	ros::Publisher bubbles2_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

	// publish robot joint values
	ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("/joint_command", 1);

	sensor_msgs::JointState (*to_joint_state)(const Vector& _q);

	if (robot.get_name()=="panda")
		to_joint_state = &to_joint_state_panda;
	else
		to_joint_state = &to_joint_state_ur10;

	static tf::TransformBroadcaster br;

	Vector q = q0;


	ros::Rate rate(RATE);
	double t=0;

	while (node.ok()) {

		obs_pub.publish(obs_marker);

		joint_pub.publish(to_joint_state(q));

		if (t<=topt) {
			t += 0.001;
			if (t>topt) t=topt;
		}

		if (space==LocalPlanner::JOINT) {
			// Interpolation in joint space
			q = q0 + t*(q1-q0);
		} else {
			// Interpolation in Cartesian space
			tf::Transform xt(x0.getRotation().slerp(x1.getRotation(),t), x0.getOrigin().lerp(x1.getOrigin(),t));
			q=robot.kdl.inverse(q,xt);
		}

		// note: already published by joint_state_publisher
		//br.sendTransform(tf::StampedTransform(robot.kdl.forward(q),ros::Time::now(), world_name, tool_name));

		//mesh_pub.publish(mesh_marker);

		//visualization_msgs::MarkerArray array = robot.tool_mesh.markers_leaves(marker_id, tool_frame_name, 1/RATE);
		size_t marker1_size=0;

		visualization_msgs::MarkerArray bubbles1 = robot.mesh(link_hit).markers_active_balls(marker_id, t, robot.fk.link_name(link_hit), 1/RATE);
		bubbles1_pub.publish(bubbles1);
		marker1_size = bubbles1.markers.size();

		//visualization_msgs::MarkerArray bubbles =robot.tool_mesh.markers_balls_level(marker_id, 0, tool_frame_name, 1/RATE);
		visualization_msgs::MarkerArray bubbles2 = obstacle.markers_active_balls(marker_id + marker1_size, t, world_name, 1/RATE);
		bubbles2_pub.publish(bubbles2);

		ros::spinOnce();
		rate.sleep();
	}

	for (int i=0; i<robot.fk.nb_links; i++) {
		if (robot.has_collision(i))
			robot.mesh(i).cleanup();
	}

	obstacle.cleanup();
}
