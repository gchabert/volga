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

#include <volga_core/UnitQuaternion.h>

using namespace std;
using namespace ibex;

namespace volga_core {

UnitQuaternion UnitQuaternion::identity() {
	return UnitQuaternion(Interval::zero(),Interval::zero(),Interval::zero(),Interval::one(),false);
}

UnitQuaternion::UnitQuaternion(const ExprNode& x, const ExprNode& y, const ExprNode& z, const ExprNode& w, bool normalize) :
	Quaternion(normalize? (const Quaternion&) Quaternion(x,y,z,w).normalize() : Quaternion(x,y,z,w)) {
}

UnitQuaternion::UnitQuaternion(const Interval& x, const Interval& y, const Interval& z, const Interval& w, bool normalize) :
		Quaternion(normalize? (const Quaternion&) Quaternion(x,y,z,w).normalize() : Quaternion(x,y,z,w)) {
}

UnitQuaternion::UnitQuaternion(const ibex::IntervalVector& x, bool normalize) :
		Quaternion(normalize? (const Quaternion&) Quaternion(x).normalize() : Quaternion(x)) {
}

UnitQuaternion::UnitQuaternion(const UnitQuaternion& q) : Quaternion(q) {

}

UnitQuaternion::UnitQuaternion(const Quaternion& q, bool normalize) :
		Quaternion(normalize? (const Quaternion&) q.normalize() : q) {
}

const ExprNode& UnitQuaternion::rotate(const ExprNode& e) const {
	// optimization of ((*this) * x) * conjugate() from wikipedia

	if (constant()) {
		Interval t2 =   c[3]*c[0];
		Interval t3 =   c[3]*c[1];
		Interval t4 =   c[3]*c[2];
		Interval t5 =  -pow(c[0],2);
		Interval t6 =   c[0]*c[1];
		Interval t7 =   c[0]*c[2];
		Interval t8 =  -pow(c[1],2);
		Interval t9 =   c[1]*c[2];
		Interval t10 = -pow(c[2],2);
		const ExprNode& y0 = 2*( (t8 + t10)*e[0] + (t6 -  t4)*e[1] + (t3 + t7)*e[2] ) + e[0];
		const ExprNode& y1 = 2*( (t4 +  t6)*e[0] + (t5 + t10)*e[1] + (t9 - t2)*e[2] ) + e[1];
		const ExprNode& y2 = 2*( (t7 -  t3)*e[0] + (t2 +  t9)*e[1] + (t5 + t8)*e[2] ) + e[2];

		return ExprVector::new_col(y0, y1, y2);
	} else {
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
}

const ExprNode& UnitQuaternion::rotate(const urdf::Vector3& v) const {

	if (constant()) {
		Interval t2 =   c[3]*c[0];
		Interval t3 =   c[3]*c[1];
		Interval t4 =   c[3]*c[2];
		Interval t5 =  -pow(c[0],2);
		Interval t6 =   c[0]*c[1];
		Interval t7 =   c[0]*c[2];
		Interval t8 =  -pow(c[1],2);
		Interval t9 =   c[1]*c[2];
		Interval t10 = -pow(c[2],2);
		Interval y0 = 2*( (t8 + t10)*v.x + (t6 -  t4)*v.y + (t3 + t7)*v.z ) + v.x;
		Interval y1 = 2*( (t4 +  t6)*v.x + (t5 + t10)*v.y + (t9 - t2)*v.z ) + v.y;
		Interval y2 = 2*( (t7 -  t3)*v.x + (t2 +  t9)*v.y + (t5 + t8)*v.z ) + v.z;

		return ExprConstant::new_vector(IntervalVector{{y0, y1, y2}},false);
	} else {

		const ExprNode& t2 =   w()*x();
		const ExprNode& t3 =   w()*y();
		const ExprNode& t4 =   w()*z();
		const ExprNode& t5 =  -pow(x(),2);
		const ExprNode& t6 =   x()*y();
		const ExprNode& t7 =   x()*z();
		const ExprNode& t8 =  -pow(y(),2);
		const ExprNode& t9 =   y()*z();
		const ExprNode& t10 = -pow(z(),2);
		const ExprNode& y0 = 2*( (t8 + t10)*v.x + (t6 -  t4)*v.y + (t3 + t7)*v.z ) + v.x;
		const ExprNode& y1 = 2*( (t4 +  t6)*v.x + (t5 + t10)*v.y + (t9 - t2)*v.z ) + v.y;
		const ExprNode& y2 = 2*( (t7 -  t3)*v.x + (t2 +  t9)*v.y + (t5 + t8)*v.z ) + v.z;

		return ExprVector::new_col(y0, y1, y2);
	}
}

UnitQuaternion::UnitQuaternion(const Quaternion& q) : Quaternion(q) {

}

UnitQuaternion UnitQuaternion::conjugate() const {
	return UnitQuaternion(Quaternion::conjugate());
}

UnitQuaternion UnitQuaternion::operator*(const UnitQuaternion& q2) const {
	return UnitQuaternion(((const Quaternion&) (*this)) * q2);
}

UnitQuaternion UnitQuaternion::copy(ExprCopy& c, const Array<const ExprSymbol>& old_args, const Array<const ExprSymbol>& new_args, bool shared) const {
	if (constant()) {
		return UnitQuaternion(*this);
	} else {
		const ExprNode& x2=c.copy(old_args, new_args, x(), shared);
		const ExprNode& y2=c.copy(old_args, new_args, y(), shared);
		const ExprNode& z2=c.copy(old_args, new_args, z(), shared);
		const ExprNode& w2=c.copy(old_args, new_args, w(), shared);
		return UnitQuaternion(x2,y2,z2,w2,false);
	}
}

const ibex::ExprNode& UnitQuaternion::stack_with(const ibex::ExprNode& translation) const {

	if (constant()) {
		/* --> no because of possible leak
		if (translation.type_id()==ExprNode::NumExprConstant) {
			const IntervalVector& t=((const ExprConstant&) translation).get_vector_value();
			IntervalVector qt(7);
			qt.put(0,c);
			qt.put(4,t);
			return ExprConstant::new_vector(qt,false);
		} else { */
			return ExprVector::new_col(
				Array<const ExprNode>(ExprConstant::new_vector(c,false), translation));
		/* } */
	} else
		return ExprVector::new_col(
			Array<const ExprNode>(x(), y(), z(), w(), translation));
}

} /* namespace volga_core */
