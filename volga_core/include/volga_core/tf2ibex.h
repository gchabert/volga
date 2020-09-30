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

#ifndef __VOLGA_CORE_TF_2_IBEX_H__
#define __VOLGA_CORE_TF_2_IBEX_H__

#include <ibex_IntervalMatrix.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>

#include "Point3D.h"

namespace volga_core {

/**
 * \brief Convert a tf quaternion to an Ibex vector
 */
ibex::IntervalVector _ibex(const tf::Quaternion& q);

/**
 * \brief Convert a tf vector to a point
 */
Point3D _ibex(const tf::Vector3& v);

/**
 * \brief Convert a tf rotation matrix to an Ibex matrix
 */
ibex::Matrix _ibex(const tf::Matrix3x3& R);

/**
 * \brief Convert a tf vector to an Ibex vector
 */
tf::Vector3 _tf(const ibex::Vector& v);

/**
 * \brief Convert an ibex constant matrix to a tf matrix
 */
tf::Matrix3x3 _tf(const ibex::Matrix& R);

/**
 * \brief Convert a vector representing a transformation
 * to a tf transformation
 *
 * The vector must be of size 7 and structured
 * as (q,x) where q=quaternion and x=point
 */
tf::Transform _tf_transform(const ibex::Vector& v);

/**
 * \brief Convert an homogeneous 3x4 matrix to a tf transformation
 */
tf::Transform _tf_transform(const ibex::Matrix& M);

/**
 * \brief Display a tf transformation
 */
std::ostream& operator<<(std::ostream& os, const tf::Transform& t);

/*============================================ inline implementation ============================================ */

inline ibex::IntervalVector _ibex(const tf::Quaternion& q) {
	return {
		{q.getX(),q.getY(),q.getZ(),q.getW()}
	};
}

inline Point3D _ibex(const tf::Vector3& v) {
	return {
		{v.getX(),v.getY(),v.getZ()}
	};
}

inline ibex::Matrix _ibex(const tf::Matrix3x3& R) {
	return {
		{R[0][0],R[0][1],R[0][2]},
		{R[1][0],R[1][1],R[1][2]},
		{R[2][0],R[2][1],R[2][2]}
	};
}

inline tf::Vector3 _tf(const ibex::Vector& v) {
	assert(v.size()==3);
	return tf::Vector3(v[0],v[1],v[2]);
}

inline tf::Matrix3x3 _tf(const ibex::Matrix& R) {
	assert(R.nb_cols()==3);
	assert(R.nb_rows()==3);
	return tf::Matrix3x3(
			R[0][0],R[0][1],R[0][2],
			R[1][0],R[1][1],R[1][2],
			R[2][0],R[2][1],R[2][2]
	);
}

inline tf::Transform _tf_transform(const ibex::Vector& v) {
	return tf::Transform(
			tf::Quaternion(v[0],v[1],v[2],v[3]),
			_tf(v.subvector(4,6))
	);
}

inline tf::Transform _tf_transform(const ibex::Matrix& M) {
	return tf::Transform(
			_tf(M.submatrix(0,2,0,2)),
			_tf(M.col(3)));
}

} /* namespace volga_core */

#endif /* __VOLGA_CORE_TF_2_IBEX_H__ */
