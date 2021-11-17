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
#ifndef __VOLGA_CORE_ROTATION_H__
#define __VOLGA_CORE_ROTATION_H__

#include <urdf/model.h>
#include <ibex_ExprCopy.h>
#include "UnitQuaternion.h"

namespace volga_core {

/**
 * \brief Ibex representation of a quaternion
 */
class Rotation {
public:

	/**
	 * \brief Build a rotation from a quaternion
	 */
	Rotation(const UnitQuaternion& q);

	/**
	 * \brief Build a rotation around an axis
	 */
	Rotation(const urdf::Vector3& axis, const ibex::ExprNode& qi, bool use_quaternions);

	/**
	 * \brief Delete this
	 */
	~Rotation();

	/**
	 * \brief Add all the nodes (for cleanup)
	 */
	void add_nodes(std::vector<const ibex::ExprNode*>& nodes) const;

	/**
	 * \brief Copy (share expressions)
	 */
	Rotation(const Rotation& r);

	/**
	 * \brief Assign to
	 */
	Rotation& operator=(const Rotation& r);

	/**
	 * \brief Identity rotation
	 */
	static Rotation identity(bool use_quaternions);

	/**
	 * \brief Build a rotation from a matrix
	 */
	Rotation(const ibex::ExprNode& m);

	/**
	 * \brief Create the quaternion from the rotation matrix
	 */
	UnitQuaternion quaternion() const;

	/**
	 * \brief Return the transformation expression
	 *
	 * The returned transformation (rotation plus translation)
	 * can either be a 3x4 matrix or a 7x1 vector.
	 */
	const ibex::ExprNode& stack_with(const ibex::ExprNode& translation) const;

	/**
	 * \brief Rotate a vector
	 */
	const ibex::ExprNode& operator*(const ibex::ExprNode& x) const;

	/**
	 * \brief Rotate a vector
	 */
	const ibex::ExprNode& operator*(const urdf::Vector3& x) const;

	/**
	 * \brief Compose with another rotation
	 */
	Rotation operator*(const Rotation& r) const;

	/**
	 * \brief Compose with another rotation
	 */
	Rotation operator*(const urdf::Rotation& r) const;

	/**
	 * \brief Copy expressions
	 */
	Rotation copy(ibex::ExprCopy& c, const ibex::Array<const ibex::ExprSymbol>& old_x, const ibex::Array<const ibex::ExprSymbol>& new_x, bool shared=false) const;

protected:
	friend std::ostream& operator<<(std::ostream& os, const Rotation& r);

	/*
	 * Build a rotation from URDF
	 */
	Rotation(const urdf::Rotation& r, bool use_quaternions);

	/*
	 * NULL if use a quaternion is used (q!=NULL)
	 */
	const ibex::ExprNode* matrix;

	/*
	 * NULL if a matrix is used (matrix!=NULL)
	 */
	const UnitQuaternion* q;
};

/**
 * \brief Display a rotation
 */
std::ostream& operator<<(std::ostream& os, const Rotation& r);

} /* namespace volga_core */


#endif /* __VOLGA_CORE_ROTATION_H__ */
