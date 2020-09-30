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

#include <ibex_IntervalVector.h>
#include <ibex_ExprOperators.h>
#include <volga_core/Ball.h>
#include <volga_core/Triangle.h>

using namespace std;
using namespace ibex;

namespace volga_core {

Interval sqnorm(const IntervalVector& d) {
	return pow(d[0],2)+pow(d[1],2)+pow(d[2],2);
}

Ball::Ball(const Point3D& center, double radius) : center(center), radius(radius), volume(coeff_volume * std::pow(radius,3)) {

}

Ball::Ball(const Ball& b) : center(b.center), radius(b.radius), volume(b.volume) {
}

Ball::Ball(const Ball& b1, const Ball& b2, bool validate) {

	if (b1.NV_is_subset(b2)) *this=b2;
	else if (b2.NV_is_subset(b1)) *this=b1;
	else {
		double d=distance(b1.center,b2.center);
		if (d>1e-10) {
			radius = 0.5*(b1.radius + b2.radius + d);
			center = b1.center + (radius - b1.radius)/d*(b2.center-b1.center);
		} else {
			radius = std::max(b1.radius,b2.radius) + 0.5*d;
			center = 0.5*(b1.center + b2.center);
		}
		volume = coeff_volume * std::pow(radius,3);
	}

	// validate radius
	// use simple inflation technique
	if (validate) {
		double total=1;
		while ( ((IntervalVector(b1.center)-center).norm2() + b1.radius).ub() > radius ||
				((IntervalVector(b2.center)-center).norm2() + b2.radius).ub() > radius ) {
			radius *= 1.001;
			total *= 1.001;
		}
		// a total>1.001 means 2 successive inflations: should be rare!
		if (total>1.001) cout << "[ball union] inflation by " << ((total-1)*100.) << "%\n";
	}
}


void Ball::validate_radius(const Triangle& tr, const Point3D& center, double& radius, bool circumsphere) {
	int v=0; // count vertex located at the boundary of the circle.
	for (int i=0; i<3; i++) {
		double normi=(IntervalVector(tr.vertices[i])-center).norm2().ub();
		if (normi>radius) {
			if (normi>1.01*radius) cerr << "warning: radius by enlarged more than 1%\n";
			radius=normi;
			v++;
		} else if (normi>0.99*radius) {
			v++;
		} else {
			//if (circumsphere) cerr << "delta= " << normi << " " << radius << endl;
		}
	}
	if ((circumsphere && v<3) || v<2) cerr << "warning: only " << v << " vertex at the boundary !\n";
}


Ball::Ball(const Triangle& tr) {

	IntervalVector a=tr.vertices[0];
	IntervalVector b=tr.vertices[1];
	IntervalVector c=tr.vertices[2];
	IntervalVector ac = c - a ;
	IntervalVector ab = b - a ;
	IntervalVector bc = c - b ;

	double AB=ab.norm2().ub();
	double AC=ac.norm2().ub();
	double BC=bc.norm2().ub();

	double smallest_seg_length=AB;
	double greatest_seg_length=AB;
	Vector greatest_seg_center=0.5*(a+b).mid();

	if (AC>greatest_seg_length) {
		greatest_seg_length = AC;
		greatest_seg_center = 0.5*(a+c).mid();
	} else if (AC<smallest_seg_length) {
		smallest_seg_length = AC;
	}

	if (BC>greatest_seg_length) {
		greatest_seg_length = BC;
		greatest_seg_center = 0.5*(b+c).mid();
	} else if (BC<smallest_seg_length) {
		smallest_seg_length = BC;
	}

	double scal01 = ((b-a)*(c-a)).lb();
	double scal02 = ((c-b)*(a-b)).lb();
	double scal03 = ((a-c)*(b-c)).lb();

	if (smallest_seg_length<0.1*greatest_seg_length || scal01<0 || scal02<0 || scal03<0) {
		//cout << "AB.AC<0" << endl;
		center = greatest_seg_center;
		radius = greatest_seg_length/2.0;
		validate_radius(tr, center, radius, false);
		volume = coeff_volume * std::pow(radius,3);
		return;
	}

	IntervalVector abXac = cross(ab,ac);

	// copy/paste from https://gamedev.stackexchange.volume = coeff_volume * std::pow(radius,3);com/questions/60630/how-do-i-find-the-circumcenter-of-a-triangle-in-3d
	// ===========================================================
	// this is the vector from a TO the circumsphere center
	//cout << "2.*abXac.sqnorm2()=" << 2.*abXac.sqnorm2()<< " scal02=" << scal02 << " scal03=" << scal03 << endl;
	IntervalVector  toCircumsphereCenter = (Interval::ONE/(2.*abXac.sqnorm2())) * (ac.sqnorm2()*cross(abXac,ab) + ab.sqnorm2()*cross(ac,abXac));
	radius = toCircumsphereCenter.norm2().ub();

	// The 3 space coords of the circumsphere center then:
	center = (a  +  toCircumsphereCenter).mid();
	validate_radius(tr, center, radius, true);

	// ===========================================================
	volume = coeff_volume * std::pow(radius,3);
}

bool Ball::NV_contains(const Point3D& p) const {
	return distance(p,center) <= radius;
}

bool Ball::NV_is_subset(const Ball& b2) const {
	return distance(center,b2.center)+radius <= b2.radius;
}

double Ball::delta_volume(const Ball& b1, const Ball& b2) {
	double radius_union = 0.5*(b1.radius + b2.radius + distance(b1.center,b2.center));
	return Ball::coeff_volume*(std::pow(radius_union,3)-std::pow(b1.radius,3)-std::pow(b2.radius,3));
}

visualization_msgs::Marker Ball::marker(int32_t id, const string& marker_frame_id, Ball::ball_color color, double lifetime) const {
	visualization_msgs::Marker marker;
	marker.header.frame_id = marker_frame_id;
	marker.ns = "volga";
	marker.id = id;

	// Set the marker type.
	marker.type = visualization_msgs::Marker::SPHERE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = center.x();
	marker.pose.position.y = center.y();
	marker.pose.position.z = center.z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker (*2 because we need diameter)
	marker.scale.x = 2.0*radius;
	marker.scale.y = 2.0*radius;
	marker.scale.z = 2.0*radius;

	// Set the color -- be sure to set alpha to something non-zero!
	switch (color) {
	case COLLISION:
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;
		break;
	case FREE:
		marker.color.r = 0.9;
		marker.color.g = 0.9;
		marker.color.b = 0.9;
		marker.color.a = 0.3;
		break;
	case NEXT_POPPED:
		marker.color.r = 0.2;
		marker.color.g = 1.0;
		marker.color.b = 0.2;
		marker.color.a = 0.3;
		break;
	case OBSTACLE:
		marker.color.r = 0.4;
		marker.color.g = 0.4;
		marker.color.b = 1.0;
		marker.color.a = 1.0;
	}
	marker.lifetime = ros::Duration(lifetime);
	return marker;
}

ostream& operator<<(ostream& os, const Ball& ball) {
	return os << "center=" << ball.center << " radius=" << ball.radius;
}

} /* namespace volga_core */
