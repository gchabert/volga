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

#ifndef  __TRIANGLE_H__
#define  __TRIANGLE_H__

#include "Ball.h"
#include "Point3D.h"

namespace volga_core {

/**
 * \brief Triangle.
 */
class Triangle {
public:

	/**
	 * \brief Build a triangle from vertices
	 */
	Triangle(const Point3D& v1, const Point3D& v2, const Point3D& v3);

	/**
	 * \brief Vertices
	 */
	Point3D vertices[3];

	/**
	 * \brief Enclosing ball of the triangle.
	 *
	 * The center is only approximate but the radius is
	 * slightly augmented so that the ball is ensured to
	 * enclose the triangle.
	 */
	Ball enclosing_ball;
};

/**
 * \brief Display a triangle
 */
std::ostream& operator<<(std::ostream&, const Triangle& tr);

/*============================================ inline implementation ============================================ */

inline Triangle::Triangle(const Point3D& v1, const Point3D& v2, const Point3D& v3) : vertices{v1,v2,v3}, enclosing_ball(*this) {
}

} /* namespace volga_core */

#endif /* __VOLGA_CORE_TRIANGLE_H_ */
