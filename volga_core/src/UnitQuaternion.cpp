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

#include <volga_core/UnitQuaternion.h>

using namespace std;
using namespace ibex;

namespace volga_core {

UnitQuaternion UnitQuaternion::identity() {
	const ExprConstant& zero=ExprConstant::new_scalar(Interval::zero());
	const ExprConstant& one=ExprConstant::new_scalar(Interval::one());
	return UnitQuaternion(zero,zero,zero,one,false);
}

Quaternion UnitQuaternion::normalize(const ExprNode& x, const ExprNode& y, const ExprNode& z, const ExprNode& w) {
	const ExprNode& norm =sqrt(pow(x,2)+pow(y,2)+pow(z,2)+pow(w,2));
	return (1/norm)*Quaternion(x,y,z,w);
}

UnitQuaternion::UnitQuaternion(const ExprNode& x, const ExprNode& y, const ExprNode& z, const ExprNode& w, bool normalize) :
	Quaternion(normalize? UnitQuaternion::normalize(x,y,z,w) : Quaternion(x,y,z,w)) {
}

UnitQuaternion::UnitQuaternion(const Quaternion& q, bool normalize) :
		Quaternion(normalize? UnitQuaternion::normalize(q.x(),q.y(),q.z(),q.w()) : q) {
}

const ExprNode& UnitQuaternion::rotate(const ExprNode& e) const {
	// optimization of ((*this) * x) * conjugate() from wikipedia
	const ExprNode& t2 =   w()*x();
	const ExprNode& t3 =   w()*y();
	const ExprNode& t4 =   w()*z();
	const ExprNode& t5 =  -pow(x(),2);
	const ExprNode& t6 =   x()*y();
	const ExprNode& t7 =   x()*z();
	const ExprNode& t8 =  -pow(y(),2);
	const ExprNode& t9 =   y()*z();
	const ExprNode& t10 = -pow(z(),2);
	const ExprNode& y0 = 2*( (t8 + t10)*e[0] + (t6 -  t4)*e[1] + (t3 + t7)*e[2] ) + e[0];
	const ExprNode& y1 = 2*( (t4 +  t6)*e[0] + (t5 + t10)*e[1] + (t9 - t2)*e[2] ) + e[1];
	const ExprNode& y2 = 2*( (t7 -  t3)*e[0] + (t2 +  t9)*e[1] + (t5 + t8)*e[2] ) + e[2];

	return ExprVector::new_col(y0, y1, y2);
}

UnitQuaternion::UnitQuaternion(const Quaternion& q) : Quaternion(q) {

}

UnitQuaternion UnitQuaternion::conjugate() const {
	return UnitQuaternion(Quaternion::conjugate());
}

UnitQuaternion UnitQuaternion::operator*(const UnitQuaternion& q2) const {
	return UnitQuaternion(((const Quaternion&) (*this)) * q2);
}

} /* namespace volga_core */
