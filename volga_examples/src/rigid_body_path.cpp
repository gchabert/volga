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
using namespace volga_core;

#define _PACKAGE_NAME "volga_examples"

constexpr double RATE = 30;

int main(int argc, char** argv) {

	ros::init(argc, argv, "rigid_body_path");

	ros::NodeHandle node;

	Mesh mesh;
	double epsilon = 0.01;

	std::string volga_path = ros::package::getPath(_PACKAGE_NAME);
	std::string STL_filename = volga_path + "/meshes/fork_light.stl";
	mesh.init(STL_filename, false, epsilon, true);

	tf::Transform x0(tf::Transform(tf::Matrix3x3::getIdentity(), tf::Vector3(0,0,0)));
	tf::Transform x1(tf::Transform(tf::Quaternion(TFSIMD_HALF_PI,0,TFSIMD_2_PI), tf::Vector3(1,1,1)));

	FrameInterpol::set_t0_t1(x0,x1);
	Function& interp_cart=FrameInterpol::interpol();
	// put the obstacle at midpoint in the Cartesian interpolation
	// with a shift of 10 cm in z axis
	// 0.12 : pass
	// 0.13 : collision
	tf::Transform half_time_pose = _tf_transform(interp_cart.eval_vector(IntervalVector(1,Interval(0.5))).lb());
	tf::Quaternion _u_turn;
	_u_turn.setRPY(TFSIMD_PI,0,0);
	tf::Transform u_turn(_u_turn, tf::Vector3(0.02,0,0.2));

	int marker_id=1;

	string world_frame_name="world";

	Mesh obs_mesh;

	// ========= test with a simple ball as obstacle =============
//	tf::Transform shift(tf::Matrix3x3::getIdentity(), tf::Vector3(0,0,0.1));
//	Point3D x_obs = _ibex((half_time_pose*shift).getOrigin());
//	Ball obstacle(x_obs, 0.010);
//	obs_mesh.init(obstacle);
//	visualization_msgs::Marker obs_marker = obstacle.marker(marker_id++, world_frame_name, Ball::OBSTACLE, 1/RATE);
	// ===========================================================

	// ========= test with a tuning fork as obstacle =============
	obs_mesh.init(STL_filename, false, epsilon, true);
	obs_mesh.move(half_time_pose * u_turn);
	visualization_msgs::Marker obs_marker = obs_mesh.markers_facets(marker_id++, world_frame_name, 1/RATE);
	// ===========================================================

	LocalPlanner planner(mesh,obs_mesh,x0,x1);
	planner.update_mesh = true;
	//planner.log = true;

	string tuning_fork_frame_name="fork";
	visualization_msgs::Marker mesh_marker = mesh.markers_STL(marker_id++, tuning_fork_frame_name, 1/RATE);

	cout << "computing max time...\n";

	double topt = planner.max_time(epsilon);

	cout << "done! tmax=" << topt << "\n";

	// publish pose of the obstacles
	ros::Publisher obs_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// publish pose of the original mesh
	ros::Publisher mesh_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// publish pose of the active bubbles
	ros::Publisher bubble_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

	static tf::TransformBroadcaster br;

	ros::Rate rate(RATE);
	double t=0.3;

	while (node.ok()) {

		t += 0.001;
		if (t>topt) t=topt;

		Vector xt=interp_cart.eval_vector(IntervalVector(1,Interval(t))).mid();

		br.sendTransform(tf::StampedTransform(_tf_transform(xt),ros::Time::now(), world_frame_name, tuning_fork_frame_name));

		//visualization_msgs::MarkerArray array = mesh.markers_leaves(marker_id, tool_frame_name);
		visualization_msgs::MarkerArray bubbles1 = mesh.markers_active_balls(marker_id, t, tuning_fork_frame_name, 1/RATE);

		visualization_msgs::MarkerArray bubbles2 = obs_mesh.markers_active_balls(marker_id + bubbles1.markers.size(), t, world_frame_name, 1/RATE);

		obs_pub.publish(obs_marker);

		mesh_pub.publish(mesh_marker);

		bubble_pub.publish(bubbles1);

		bubble_pub.publish(bubbles2);

		ros::spinOnce();
		rate.sleep();
	}

	mesh.cleanup();
}
