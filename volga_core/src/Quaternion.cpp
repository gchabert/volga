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

#include <ibex_ExprCopy.h>
#include <volga_core/Quaternion.h>
#include <volga_core/UnitQuaternion.h>
#include <cassert>

using namespace std;
using namespace ibex;

namespace volga_core {

Quaternion::Quaternion(const ExprNode& x, const ExprNode& y, const ExprNode& z, const ExprNode& w) :
			_x(&x), _y(&y), _z(&z), _w(&w), c(1) {
}

Quaternion::Quaternion(const IntervalVector& q) :
				_x(NULL), _y(NULL), _z(NULL), _w(NULL), c(q) {

	assert(q.size()==4);
}

Quaternion::Quaternion(const Interval& x, const Interval& y, const Interval& z, const Interval& w) :
				_x(NULL), _y(NULL), _z(NULL), _w(NULL), c{{x,y,z,w}} {

}

Quaternion::Quaternion(const Quaternion& q) : _x(q._x), _y(q._y), _z(q._z), _w(q._w), c(q.c) {

}

void Quaternion::add_nodes(vector<const ExprNode*>& nodes) const {
	if (!constant()) {
		nodes.push_back(_x);
		nodes.push_back(_y);
		nodes.push_back(_z);
		nodes.push_back(_w);
	}
}

bool Quaternion::constant() const {
	return _x==NULL;
}

Quaternion operator+(const Quaternion& q1, const Quaternion& q2) {
	if (q1.constant()) {
		if (q2.constant()) {
			return Quaternion(q1.c+q2.c);
		} else {
			return Quaternion(
					q1.c[0]+q2.x(),
					q1.c[1]+q2.y(),
					q1.c[2]+q2.z(),
					q1.c[3]+q2.w());
		}
	} else {
		if (q2.constant()) {
			return Quaternion(
					q1.x()+q2.c[0],
					q1.y()+q2.c[1],
					q1.z()+q2.c[2],
					q1.w()+q2.c[3]);
		} else {
			return Quaternion(
					q1.x()+q2.x(),
					q1.y()+q2.y(),
					q1.z()+q2.z(),
					q1.w()+q2.w());
		}
	}
}

Quaternion operator-(const Quaternion& q1, const Quaternion& q2) {
	if (q1.constant()) {
		if (q2.constant()) {
			return Quaternion(q1.c-q2.c);
		} else {
			return Quaternion(
					q1.c[0]-q2.x(),
					q1.c[1]-q2.y(),
					q1.c[2]-q2.z(),
					q1.c[3]-q2.w());
		}
	} else {
		if (q2.constant()) {
			return Quaternion(
					q1.x()-q2.c[0],
					q1.y()-q2.c[1],
					q1.z()-q2.c[2],
					q1.w()-q2.c[3]);
		} else {
			return Quaternion(
					q1.x()-q2.x(),
					q1.y()-q2.y(),
					q1.z()-q2.z(),
					q1.w()-q2.w());
		}
	}
}

Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {

	if (q1.constant()) {
		if (q2.constant()) {
			return Quaternion(
					q1.c[3] * q2.c[0] + q1.c[0] * q2.c[3] + q1.c[1] * q2.c[2] - q1.c[2] * q2.c[1],
					q1.c[3] * q2.c[1] + q1.c[1] * q2.c[3] + q1.c[2] * q2.c[0] - q1.c[0] * q2.c[2],
					q1.c[3] * q2.c[2] + q1.c[2] * q2.c[3] + q1.c[0] * q2.c[1] - q1.c[1] * q2.c[0],
					q1.c[3] * q2.c[3] - q1.c[0] * q2.c[0] - q1.c[1] * q2.c[1] - q1.c[2] * q2.c[2]);
		} else {
			return Quaternion(
					q1.c[3] * q2.x() + q1.c[0] * q2.w() + q1.c[1] * q2.z() - q1.c[2] * q2.y(),
					q1.c[3] * q2.y() + q1.c[1] * q2.w() + q1.c[2] * q2.x() - q1.c[0] * q2.z(),
					q1.c[3] * q2.z() + q1.c[2] * q2.w() + q1.c[0] * q2.y() - q1.c[1] * q2.x(),
					q1.c[3] * q2.w() - q1.c[0] * q2.x() - q1.c[1] * q2.y() - q1.c[2] * q2.z());
		}
	} else {
		if (q2.constant()) {
			return Quaternion(
					q1.w() * q2.c[0] + q1.x() * q2.c[3] + q1.y() * q2.c[2] - q1.z() * q2.c[1],
					q1.w() * q2.c[1] + q1.y() * q2.c[3] + q1.z() * q2.c[0] - q1.x() * q2.c[2],
					q1.w() * q2.c[2] + q1.z() * q2.c[3] + q1.x() * q2.c[1] - q1.y() * q2.c[0],
					q1.w() * q2.c[3] - q1.x() * q2.c[0] - q1.y() * q2.c[1] - q1.z() * q2.c[2]);
		} else {
			return Quaternion(
					q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
					q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
					q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
					q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z());
		}
	}
}

UnitQuaternion Quaternion::normalize() const {
	if (constant()) {
		Interval norm =sqrt(pow(c[0],2)+pow(c[1],2)+pow(c[2],2)+pow(c[3],2));
		return UnitQuaternion(1/norm*c, false);
	} else {
		const ExprNode& norm =sqrt(pow(x(),2)+pow(y(),2)+pow(z(),2)+pow(w(),2));
		return UnitQuaternion((1/norm)*(*this), false);
	}
}

Quaternion operator*(const ExprNode& x, const Quaternion& q) {
	assert(x.dim.is_scalar());
	if (q.constant())
		return Quaternion(q.c[0]*x, q.c[1]*x, q.c[2]*x, q.c[3]*x);
	else
		return Quaternion(x*q.x(), x*q.y(), x*q.z(), x*q.w());
}

Quaternion operator*(const Interval& x, const Quaternion& q) {
	if (q.constant())
		return Quaternion(x*q.c);
	else
		return Quaternion(x*q.x(), x*q.y(), x*q.z(), x*q.w());
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
	if (q.constant())
		return os << "(" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ")";
	else
		return os << q.c;
}

} /* namespace volga_core */
