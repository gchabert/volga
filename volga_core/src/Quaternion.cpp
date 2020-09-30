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
#include <volga_core/Quaternion.h>

using namespace std;
using namespace ibex;

namespace volga_core {

Quaternion::Quaternion(const ExprNode& x, const ExprNode& y, const ExprNode& z, const ExprNode& w) :
	_x(&x), _y(&y), _z(&z), _w(&w) {
}

Quaternion operator+(const Quaternion& q1, const Quaternion& q2) {
	return Quaternion(
			q1.x()+q2.x(),
			q1.y()+q2.y(),
			q1.z()+q2.z(),
			q1.w()+q2.w());
}

Quaternion operator-(const Quaternion& q1, const Quaternion& q2) {
	return Quaternion(
			q1.x()-q2.x(),
			q1.y()-q2.y(),
			q1.z()-q2.z(),
			q1.w()-q2.w());
}

Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {
	return Quaternion(
			q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
			q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
			q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
			q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z());
}

Quaternion operator*(const ExprNode& x, const Quaternion& q) {
	assert(x.dim.is_scalar());
	return Quaternion(x*q.x(), x*q.y(), x*q.z(), x*q.w());
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
	return os << "(" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ")";
}

} /* namespace volga_core */
