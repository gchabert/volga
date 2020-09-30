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

#ifndef __VOLGA_CORE_FRAME_H__
#define __VOLGA_CORE_FRAME_H__

#include <ibex_Interval.h>
#include <ibex_TemplateDomain.h>
#include <ibex_Function.h>
#include <tf/LinearMath/Transform.h>

#include "Point3D.h"
#include "Rotation.h"

namespace volga_core {

/**
 * \brief Ibex expression of a frame
 */
class Frame {
public:
	/**
	 * \brief Create a new frame or transformation.
	 *
	 * \param position - The origin of the frame in the world frame
	 *                   (i.e., translation).
	 * \param rotation - The orientation of the frame in the world frame
	 *                   (i.e., rotation).
	 */
	Frame(const ibex::ExprNode& position, const Rotation& rotation);

	/**
	 * \brief Create a new frame or transformation
	 */
	Frame(const ibex::ExprNode& position_and_rotation);

	/**
	 * \brief Return T*x where T is this transformation
	 *
	 * Does not make a copy of expression nodes.
	 */
	const ibex::ExprNode& move(const urdf::Vector3& x) const;

	/**
	 * \brief Return T*x where T is this transformation
	 *
	 * Does not make a copy of expression nodes.
	 */
	const ibex::ExprNode& move(const ibex::ExprNode& x) const;

	/**
	 * \brief Copy the expressions
	 */
	Frame copy(const ibex::Array<const ibex::ExprSymbol>& old_q, const ibex::Array<const ibex::ExprSymbol>& new_q) const;

	/**
	 * \brief position/translation.
	 */
	const ibex::ExprNode& position;

	/**
	 * \brief orientation/rotation.
	 */
	Rotation rotation;

	/**
	 * \brief Stack the position and rotation.
	 *
	 * Stack in a single vector or matrix of expressions
	 * depending if quaternions are used or not.
	 * If quaternions are used the result is a vector, otherwise
	 * (rotation matrices are used), it is a matrix.
	 */
	const ibex::ExprNode& stack();
};

///**
// * \brief Interval constant frame
// *
// *       ---- Unused so far ----
// */
//class ConstFrame {
//public:
//	/**
//	 * \brief Create a new frame or transformation.
//	 */
//	ConstFrame(const ibex::IntervalVector& position, const ibex::IntervalMatrix& rotation);
//
//	/**
//	 * \brief Create an uninitialized frame.
//	 */
//	ConstFrame();
//
//	/**
//	 * \brief Get a tf equivalent (midpoints)
//	 */
//	tf::Transform _tf();
//
//	/**
//	 * \brief position/translation.
//	 */
//	const ibex::IntervalVector position;
//
//	/**
//	 * \brief orientation/rotation (matrix)
//	 */
//	const ibex::IntervalMatrix rotation;
//};

/**
 * \brief Display a frame
 */
std::ostream& operator<<(std::ostream& os, const Frame& frame);

/**
 * \brief Return T*x where T is this transformation
 *
 * Does not make a copy of expression nodes.
 */
const ibex::ExprNode& moveto(const tf::Transform& frame, const ibex::ExprNode& x);

} /* namespace volga_core */

#endif /* __VOLGA_CORE_FRAME_H__ */
