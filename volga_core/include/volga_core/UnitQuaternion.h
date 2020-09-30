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

#ifndef __VOLGA_CORE_UNIT_QUATERNION_H__
#define __VOLGA_CORE_UNIT_QUATERNION_H__

#include "Quaternion.h"

namespace volga_core {

/**
 * \brief Ibex expression representing a unit quaternion.
 */
class UnitQuaternion : public Quaternion {
public:

	/**
	 * \brief Build a unit quaternion.
	 *
	 * \param x
	 * \param y
	 * \param z
	 * \param w
	 * \param normalize - if true, the quaternion is normalized. If false, (x,y,z,w) is
	 * 			          assumed to be already normalized.
	 * Note: x() may not return "x" given in the constructor because of normalization.
	 *
	 */
	UnitQuaternion(const ibex::ExprNode& x, const ibex::ExprNode& y, const ibex::ExprNode& z, const ibex::ExprNode& w, bool normalize);

	/**
	 * \brief Copy and (optionally) normalize a unit quaternion.
	 *
	 * \param q         - quaternion to copy (nodes are not copied)
	 * \param normalize - if true, the quaternion is normalized. If false, q is
	 * 			          assumed to be already normalized.
	 */
	UnitQuaternion(const Quaternion& q, bool normalize);

	/**
	 *  \brief Identity quaternion
	 */
	static UnitQuaternion identity();

	/**
	 * \brief Multiply two unit quaternions.
	 */
	UnitQuaternion operator*(const UnitQuaternion& q2) const;

	/**
	 * \brief The conjugate, i.e., the inverse.
	 */
	UnitQuaternion conjugate() const;

	/**
	 * \brief Rotation by a unit quaternion.
	 *
	 * Rotate a vector \a x by the rotation that
	 * this quaternion represents.
	 */
	const ibex::ExprNode& rotate(const ibex::ExprNode& x) const;

protected:

	/*
	 * Copy
	 */
	explicit UnitQuaternion(const Quaternion& q);

	/*
	 * Normalization
	 */
	static Quaternion normalize(const ibex::ExprNode& x, const ibex::ExprNode& y, const ibex::ExprNode& z, const ibex::ExprNode& w);
};

} /* namespace volga_core */

#endif /* __VOLGA_CORE_UNIT_QUATERNION_H__ */
