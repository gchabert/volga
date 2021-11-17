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

#ifndef __VOLGA_CORE_QUATERNION_H__
#define __VOLGA_CORE_QUATERNION_H__

#include <ibex_Expr.h>
#include "Point3D.h"

namespace volga_core {

class UnitQuaternion;

/**
 * \brief Ibex expression representing a quaternion.
 */
class Quaternion {
public:

	/**
	 * \brief Create the quaternion (x,y,z,w).
	 */
	Quaternion(const ibex::ExprNode& x, const ibex::ExprNode& y, const ibex::ExprNode& z, const ibex::ExprNode& w);

	/**
	 * \brief Create the constant quaternion q.
	 */
	Quaternion(const ibex::IntervalVector& q);

	/**
	 * \brief Create the constant quaternion q=(x,y,z,w).
	 */
	Quaternion(const ibex::Interval& x, const ibex::Interval& y, const ibex::Interval& z, const ibex::Interval& w);

	/**
	 * \brief Copy a quaternion.
	 *
	 * \warning shallow copy: x, y and z transmitted by reference.
	 */
	Quaternion(const Quaternion& q);

	/**
	 * \brief Normalize this.
	 */
	UnitQuaternion normalize() const;

	/**
	 * \brief Add all the nodes (for cleanup)
	 */
	void add_nodes(std::vector<const ibex::ExprNode*>& nodes) const;

protected:

	/**
	 * \brief x component by reference.
	 */
	const ibex::ExprNode& x() const;

	/**
	 * \brief y component by reference.
	 */
	const ibex::ExprNode& y() const;

	/**
	 * \brief z component by reference.
	 */
	const ibex::ExprNode& z() const;

	/**
	 * \brief w component by reference.
	 */
	const ibex::ExprNode& w() const;

	/**
	 * \brief The conjugate
	 */
	Quaternion conjugate() const;

	friend Quaternion operator+(const Quaternion& q1, const Quaternion& q2);
	friend Quaternion operator-(const Quaternion& q1, const Quaternion& q2);
	friend Quaternion operator*(const Quaternion& q1, const Quaternion& q2);
	friend Quaternion operator*(const ibex::ExprNode& x, const Quaternion& q);
	friend Quaternion operator*(const ibex::Interval& x, const Quaternion& q);
	friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);

	/**
	 * True if this quaternion is constant
	 */
	bool constant() const;

	/**
	 * \brief x
	 *
	 * Null if constant quaternion
	 */
	const ibex::ExprNode* _x;

	/**
	 * \brief y
	 *
	 * Null if constant quaternion
	 */
	const ibex::ExprNode* _y;

	/**
	 * \brief z
	 *
	 * Null if constant quaternion
	 */
	const ibex::ExprNode* _z;

	/**
	 * \brief w
	 *
	 * Null if constant quaternion
	 */
	const ibex::ExprNode* _w;


	/**
	 * \brief The quaternion, if constant
	 *
	 * Unused if non-constant quaternion
	 */
	const ibex::IntervalVector c;

	/**
	 * \brief Multiply *this by [0,x].
	 *
	 * Useful in the rotation formula.
	 *
	 */
	Quaternion operator*(const ibex::ExprNode& x) const;
};

/**
 * \brief Add two quaternions
 */
Quaternion operator+(const Quaternion& q1, const Quaternion& q2);

/**
 * \brief Subtract two quaternions
 */
Quaternion operator-(const Quaternion& q1, const Quaternion& q2);

/**
 * \brief Multiply two quaternions
 */
Quaternion operator*(const Quaternion& q1, const Quaternion& q2);

/**
 * \brief Left multiplication by a scalar
 */
Quaternion operator*(const ibex::ExprNode& x, const Quaternion& q);

/**
 * \brief Left multiplication by a scalar
 */
Quaternion operator*(const ibex::Interval& x, const Quaternion& q);

/**
 * \brief Display a quaternion.
 */
std::ostream& operator<<(std::ostream& os, const Quaternion& q);


/*============================================ inline implementation ============================================ */

inline const ibex::ExprNode& Quaternion::x() const {
	return *_x;
}

inline const ibex::ExprNode& Quaternion::y() const {
	return *_y;
}

inline const ibex::ExprNode& Quaternion::z() const {
	return *_z;
}

inline const ibex::ExprNode& Quaternion::w() const {
	return *_w;
}

inline Quaternion Quaternion::conjugate() const {
	return constant() ? Quaternion(-x(), -y(), -z(), w()) : Quaternion(-c[0], -c[1], -c[2], c[3]);
}

} /* namespace volga_core */

#endif /* __VOLGA_CORE_QUATERNION_H__ */
