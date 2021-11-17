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

#include <volga_core/Rotation.h>

using namespace std;
using namespace ibex;

namespace volga_core {

// unused
const ExprConstant& get_cst_rotmatrix(double roll, double pitch, double yaw) {

	Interval cx    = cos(Interval(yaw));
	Interval sx    = sin(Interval(yaw));
	Interval cy    = cos(Interval(pitch));
	Interval sy    = sin(Interval(pitch));
	Interval cz    = cos(Interval(roll));
	Interval sz    = sin(Interval(roll));

	IntervalMatrix R(3,3);

	R[0][0]=cy*cz;  R[0][1]=-cx*sz + cz*sy*sx; R[0][2]=cz*cx*sy + sz*sx;

	R[1][0]=cy*sz;  R[1][1]=cz*cx + sy*sz*sx;  R[1][2]=cx*sy*sz - cz*sx;

	R[2][0]=-sy;    R[2][1]=cy*sx;             R[2][2]=cy*cx;

	return ExprConstant::new_matrix(R);
}

// unused
const ExprNode& get_rotmatrix(const ExprNode& roll, const ExprNode& pitch, const ExprNode& yaw) {

	const ExprNode& cx    = cos(yaw);
	const ExprNode& sx    = sin(yaw);
	const ExprNode& cy    = cos(pitch);
	const ExprNode& sy    = sin(pitch);
	const ExprNode& cz    = cos(roll);
	const ExprNode& sz    = sin(roll);

	const ExprVector& row1=ExprVector::new_row(cy*cz, -cx*sz + cz*sy*sx, cz*cx*sy + sz*sx);
	const ExprVector& row2=ExprVector::new_row(cy*sz, cz*cx + sy*sz*sx, cx*sy*sz - cz*sx);
	const ExprVector& row3=ExprVector::new_row(-sy, cy*sx, cy*cx);

	return ExprVector::new_col(row1,row2,row3);
}

const ExprConstant& get_cst_rotmatrix(const urdf::Rotation& q) {

	Interval x(q.x);
	Interval y(q.y);
	Interval z(q.z);
	Interval w(q.w);
	Interval d = pow(x,2)+pow(y,2)+pow(z,2)+pow(w,2);

	if (d.contains(0)) {
		ROS_ERROR("quaternion norm~0");
	}
	Interval s = Interval(2.0) / d;
	Interval xs = x * s,   ys = y * s,   zs = z * s;
	Interval wx = w * xs,  wy = w * ys,  wz = w * zs;
	Interval xx = x * xs,  xy = x * ys,  xz = x * zs;
	Interval yy = y * ys,  yz = y * zs,  zz = z * zs;
	Interval one(1.0);
	IntervalMatrix R(3,3);
	R[0][0] = one - (yy + zz);  R[0][1] = xy - wz;         R[0][2] = xz + wy;
	R[1][0] = xy + wz;          R[1][1] = one - (xx + zz); R[1][2] = yz - wx;
	R[2][0] = xz - wy;          R[2][1] = yz + wx;         R[2][2] = one - (xx + yy);

	return ExprConstant::new_matrix(R);
}

const ExprNode& get_rotmatrix(const urdf::Vector3& axis, const ExprNode& qi) {
	const ExprNode& c    = cos(qi);
	const ExprNode& s    = sin(qi);

	Interval x(axis.x);
	Interval y(axis.y);
	Interval z(axis.z);
	const ExprNode& v=Interval::one() - c;
	const ExprVector& row1=ExprVector::new_row(pow(x,2)*v + c, x*y*v - z*s,    x*z*v + y*s);
	const ExprVector& row2=ExprVector::new_row(x*y*v + z*s,    pow(y,2)*v + c, y*z*v - x*s);
	const ExprVector& row3=ExprVector::new_row(x*z*v - y*s,    y*z*v + x*s,    pow(z,2)*v + c);

	return ExprVector::new_col(row1,row2,row3);
}

UnitQuaternion* get_quaternion(const urdf::Vector3& axis, const ExprNode& qi) {
	const ExprNode& qi2=qi/2;
	const ExprNode& sin_qi2=sin(qi2);
	const ExprNode& w=cos(qi2);
	const ExprNode& x=axis.x*sin_qi2;
	const ExprNode& y=axis.y*sin_qi2;
	const ExprNode& z=axis.z*sin_qi2;
	return new UnitQuaternion(x,y,z,w,false);
}

const ExprConstant& get_cst_vector(const urdf::Vector3& v) {
	Vector p(3);
	p[0]=v.x; p[1]=v.y; p[2]=v.z;
	return ExprConstant::new_vector(p,false);
}

Rotation::Rotation(const UnitQuaternion& q) : matrix(NULL), q(new UnitQuaternion(q,false)) { }

Rotation::Rotation(const ExprNode& m) :  matrix(&m), q(NULL) { }

Rotation::Rotation(const Rotation& r) : matrix(r.matrix), q(r.q? new UnitQuaternion(*r.q,false) : NULL) { }

Rotation::Rotation(const urdf::Vector3& axis, const ExprNode& qi, bool use_quaternions) {
	if (use_quaternions) {
		matrix = NULL;
		q = get_quaternion(axis, qi);
	} else {
		matrix = &get_rotmatrix(axis, qi);
		q = NULL;
	}
}

Rotation::Rotation(const urdf::Rotation& r, bool use_quaternions) {
	if (use_quaternions) {
		matrix = NULL;
		double x,y,z,w;
		r.getQuaternion(x,y,z,w);
		q = new UnitQuaternion(x,y,z,w,false);
	} else {
		matrix = &get_cst_rotmatrix(r);
		q = NULL;
	}
}

UnitQuaternion Rotation::quaternion() const {
	if (q) return *q;
	else {
		const ExprNode& w=0.5*sqrt(1+(*matrix)[0][0]+(*matrix)[1][1]+(*matrix)[2][2]);
		const ExprNode& x=((*matrix)[2][1]-(*matrix)[1][2])/(4*w);
		const ExprNode& y=((*matrix)[0][2]-(*matrix)[2][0])/(4*w);
		const ExprNode& z=((*matrix)[1][0]-(*matrix)[0][1])/(4*w);
		return UnitQuaternion(x,y,z,w,false);
	}
}

Rotation& Rotation::operator=(const Rotation& r) {
	if (q) delete q; // don't delete the matrix, it's a real node.
	matrix = r.matrix;
	q = r.q? new UnitQuaternion(*r.q,false) : NULL;
	return *this;
}

Rotation::~Rotation() {
	if (q) delete q;
}

void Rotation::add_nodes(vector<const ExprNode*>& nodes) const {
	if (q)
		q->add_nodes(nodes);
	else
		nodes.push_back(matrix);
}

const ExprNode& Rotation::stack_with(const ExprNode& translation) const {
	if (q)
		return q->stack_with(translation);
	else
		return ExprVector::new_row(*matrix, translation);
}

Rotation Rotation::identity(bool use_quaternions) {
	if (use_quaternions) {
		return Rotation(UnitQuaternion::identity());
	} else {
		return Rotation(ExprConstant::new_matrix(Matrix::eye(3)));
	}
}

Rotation Rotation::operator*(const Rotation& r) const {
	if (matrix) {
		assert(r.matrix);
		return Rotation((*matrix)*(*r.matrix));
	} else {
		assert(r.q);
		return Rotation((*q)*(*r.q));
	}
}

Rotation Rotation::operator*(const urdf::Rotation& r) const {
	return (*this)*Rotation(r,q!=NULL);
}

const ExprNode& Rotation::operator*(const ExprNode& x) const {
	if (q) return q->rotate(x);
	else return (*matrix)*x;
}

const ExprNode& Rotation::operator*(const urdf::Vector3& x) const {
	if (q) return q->rotate(x);
	else return (*this) * get_cst_vector(x);
}

Rotation Rotation::copy(ExprCopy& c, const Array<const ExprSymbol>& old_args, const Array<const ExprSymbol>& new_args, bool shared) const {
	if (q)
		return q->copy(c, old_args, new_args, shared);
	else
		return c.copy(old_args, new_args, *matrix, shared);
}

std::ostream& operator<<(std::ostream& os, const Rotation& r) {
	if (r.q)
		return os << *r.q;
	else
		return os << *r.matrix;
}

} /* namespace volga_core */
