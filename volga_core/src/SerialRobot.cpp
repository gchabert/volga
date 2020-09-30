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
#include <ibex_ExprCopy.h>
#include <ros/package.h>
#include <volga_core/SerialRobot.h>
#include <volga_core/tf2ibex.h>
#include <volga_core/tf2ibex.h>

using namespace std;
using namespace ibex;

namespace volga_core {

/*
 * Note: I did not find such function in ros package.h, why?
 */
string full_path(const string& ros_path) {
	bool fail=false;
	size_t i=ros_path.find("://");
	if (i==string::npos) return ros_path;
	else {
		string file_or_package_string=ros_path.substr(0, i);
		string relative_path=ros_path.substr(i+3);
		if (file_or_package_string=="file") return relative_path;
		else if (file_or_package_string=="package") {
			int j=relative_path.find_first_of('/');
			if (j==string::npos) fail=true;
			else {
				string package_name=relative_path.substr(0,j);
				relative_path = relative_path.substr(j);
				return ros::package::getPath(package_name) + relative_path;
			}
		} else
			fail=true;
	}

	if (fail) {
		ROS_ERROR_STREAM("Cannot convert \"" << ros_path << "\" to standard path");
		exit(1);
	}
}

SerialRobot::SerialRobot(const std::string& urdf, double eps, bool param, bool use_quaternions, bool log) : _has_collision(NULL), epsilon(eps) {
	 urdf::Model model;

	if (param) {
		 if (!model.initParam(urdf)) {
			ROS_ERROR_STREAM("Failed to parse urdf parameter " << urdf);
			exit(1);
		}
	} else {
		if (!model.initFile(urdf)) {
			ROS_ERROR_STREAM("Failed to parse urdf file " << urdf);
			exit(1);
		}
	}

	init(model, eps, use_quaternions, log);
}

SerialRobot::SerialRobot(const urdf::ModelInterface& urdf, double eps, bool use_quaternions, bool log) : _has_collision(NULL), epsilon(eps) {
	init(urdf, eps, use_quaternions, log);
}

void SerialRobot::init(const urdf::ModelInterface& model, double eps, bool use_quaternions, bool log) {

	fk.init(model, use_quaternions, log);

	name = model.getName();

	_has_collision = new bool[fk.nb_links];

	unsigned int l=0;
	for (map<string, urdf::LinkSharedPtr>::const_iterator it=model.links_.begin(); it!=model.links_.end(); ++it) {
		link_numbers.insert_new(it->second->name.c_str(),l);

		urdf::LinkConstSharedPtr link=model.getLink(it->second->name);

		link_meshes.push_back(Mesh());

		// TODO: check  link->collision_array
		if (!link->collision) {
			ROS_WARN_STREAM("No collision defined for \"" << link->name << "\"\n");
			_has_collision[l++]=false;
			continue;
		}

		urdf::GeometryConstSharedPtr collision = link->collision->geometry;

		if (collision->type!=urdf::Geometry::MESH) {
			ROS_WARN_STREAM("Not implemented: box/sphere/cylinder collision geometry. Link \"" << link->name << "\" ignored.");
			_has_collision[l++]=false;
			continue;
		}

		if (log)
			ROS_INFO_STREAM("Processing collision for \"" << link->name << "\"...");

		link_meshes.back().init(full_path(((const urdf::Mesh&) *collision).filename), true, eps, log);
		_has_collision[l++]=true;
	}

	kdl.init(model, fk.base_name, fk.tool_name, fk.nb_actuated);

	// ======== build interpolation functions ==================
	if (log) cout << "Building interpolation functions..." << endl;
	for (int i=0; i<fk.nb_links; i++) {
		_fk_interp.push_back(new FKInterpol(fk(i),log));
	}

	if (log) cout << "Done." << endl << endl;
}

SerialRobot::~SerialRobot() {
	if (_has_collision) delete[] _has_collision;
	while (!_fk_interp.empty()) {
		delete _fk_interp.back();
		_fk_interp.pop_back();
	}
}

unsigned int SerialRobot::link(const string& name) {
	return link_numbers[name.c_str()];
}

Mesh& SerialRobot::mesh(unsigned int link_number) {
	return link_meshes[link_number];
}


bool SerialRobot::has_collision(uint link) const {
	return _has_collision[link];
}

FKInterpol& SerialRobot::fk_interp(uint link) {
	return *_fk_interp[link];
}

string SerialRobot::get_name() const {
	return name;
}

} // end namespace volga_core
