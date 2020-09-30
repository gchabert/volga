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

#include <volga_core/Frame.h>
#include <volga_core/tf2ibex.h>

using namespace std;
using namespace ibex;

namespace volga_core {

Frame::Frame(const ExprNode& position,const Rotation& rotation) : position(position), rotation(rotation) { }

//ConstFrame::ConstFrame(const IntervalVector& position, const IntervalMatrix& rotation) : position(position), rotation(rotation) { }
//
//ConstFrame::ConstFrame() : position(Vector::zeros(3)), rotation(Matrix::eye(3)) { }
//
//const ExprNode& Frame::operator*(const Point3D& x) {
//	return position + rotation*x;
//}
//
//tf::Transform ConstFrame::_tf() {
//	Matrix R=rotation.mid();
//	tf::Matrix3x3 _R(R[0][0],R[0][1],R[0][2],
//			R[1][0],R[1][1],R[1][2],
//			R[2][0],R[2][1],R[2][2]);
//
//	Vector v=position.mid();
//	tf::Vector3 _v(v[0],v[1],v[2]);
//	return tf::Transform(_R,_v);
//}

Frame::Frame(const ExprNode& T) :
								position(T.dim.size()==7 ?
										T[DoubleIndex::subcol(T.dim,4,6,0)] :
										T[DoubleIndex::one_col(T.dim,3)]),

								rotation(T.dim.size()==7 ?
										Rotation(UnitQuaternion(T[0], T[1], T[2], T[3], false)) :
										Rotation(T[DoubleIndex::submatrix(T.dim,0,2,0,2)])) {

}

const ExprNode& Frame::move(const urdf::Vector3& x) const {
	return position + rotation*x;
}

const ExprNode& Frame::move(const ExprNode& x) const {
	return position + rotation*x;
}

const ExprNode& moveto(const tf::Transform& frame, const ExprNode& x) {
	const ExprConstant& v=ExprConstant::new_vector(_ibex(frame.getOrigin()),false);
	const ExprConstant& R=ExprConstant::new_matrix(_ibex(frame.getBasis()));
	return v + R*x;
}

Frame Frame::copy(const Array<const ExprSymbol>& old_q, const Array<const ExprSymbol>& new_q) const {
	ExprCopy c;
	Rotation rotation_copy = rotation.copy(c, old_q, new_q, true);
	const ExprNode& position_copy = c.copy(old_q, new_q, position, true);
	return Frame(position_copy, rotation_copy);
}

const ExprNode& Frame::stack() {
	return rotation.stack_with(position);
}

ostream& operator<<(ostream& os, const Frame& frame) {
	os << "pos=" << frame.position << " rot=" << frame.rotation;
	return os;
}

} /* namespace volga_core */
