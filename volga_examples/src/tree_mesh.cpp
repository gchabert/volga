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

#include <ibex.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <volga_core/Mesh.h>

using namespace std;
using namespace ibex;
using namespace volga_core;

#define _PACKAGE_NAME "volga_examples"

constexpr double RATE = 1;

int main(int argc, char** argv) {

	ros::init(argc, argv, "tree_mesh");

	ros::NodeHandle node;

	// publish balls
	ros::Publisher bubble_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
	ros::Publisher mesh_pub   = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	Mesh mesh;

	std::string volga_path = ros::package::getPath(_PACKAGE_NAME);
	std::string STL_filename = volga_path + "/meshes/fork_light.stl";

	mesh.init(STL_filename, false, 0.02, true);

	ros::Rate rate(RATE);
	string world_frame_name="/world";
	string mesh_frame_name="/mesh_frame";

	visualization_msgs::Marker mesh_marker = mesh.markers_STL(0, mesh_frame_name, 1/RATE);

	static tf::TransformBroadcaster br;

	int level=0;

	while (node.ok()) {
		cout << "level=" << level << endl;
		if (level>mesh.max_level()) level=0; // loop

		br.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),ros::Time::now(), world_frame_name, mesh_frame_name));

		visualization_msgs::MarkerArray array = mesh.markers_balls_level(1, level++, mesh_frame_name, 1/RATE);

		mesh_pub.publish(mesh_marker);
		bubble_pub.publish(array);
		rate.sleep();
	}
}
