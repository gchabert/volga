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

#ifndef  __BALL_H__
#define  __BALL_H__

#include <visualization_msgs/Marker.h>

#include "Point3D.h"

namespace volga_core {

class Triangle;

/**
 * \brief 3D Ball.
 */
class Ball {
public:

	/**
	 * \brief Creates a ball.
	 */
	Ball(const Point3D& center, double radius);

	/**
	 * \brief **Validated** ball enclosing a triangle.
	 */
	Ball(const Triangle& tr);

	/**
	 * \brief Copy of a ball.
	 */
	Ball(const Ball& b);

	/*
	 * \brief **Validated** Union of 2 balls.
	 */
	Ball(const Ball& b1, const Ball& b2, bool validate);

	/**
	 * \brief Volume difference between two balls [non-validated]
	 */
	static double delta_volume(const Ball& b1, const Ball& b2);

	/**
	 * \brief non-validated inclusion test.
	 */
	bool NV_contains(const Point3D& p) const;

	/**
	 * \brief non-validated subset test.
	 */
	bool NV_is_subset(const Ball& b2) const;

	/**
	 * \brief Type of ball colors (for RVIZ)
	 */
	typedef enum { NEXT_POPPED, FREE, COLLISION, OBSTACLE } ball_color;

	/**
	 * \brief Get a visual marker of the ball (for RVIZ)
	 *
	 * \param id              - identifying number
	 * \param marker_frame_id - corresponding frame
	 * \param color           - color (default is FREE)
	 * \param lifetime        - lifetime in seconds (-1=always)
	 */
	visualization_msgs::Marker marker(int32_t id, const std::string& marker_frame_id, ball_color color=FREE, double lifetime=-1) const;

	/**
	 * \brief Center
	 */
	Point3D center;

	/**
	 * \brief Radius
	 */
	double radius;

	/**
	 * \brief Non-validated volume
	 */
	double volume;

protected:

	/*
	 * Factor to be applied to the radius to get the volume
	 */
	static constexpr double coeff_volume = 4./3.*3.141592653589793;

	/*
	 * Update the radius so that the ball is enclosing the triangle
	 * in a validated way.
	 */
	static void validate_radius(const Triangle& tr, const Point3D& center, double& radius, bool circumsphere);
};

/**
 * \brief Display a ball.
 */
std::ostream& operator<<(std::ostream& os, const Ball& ball);

} /* namespace volga_core */

#endif /* __VOLGA_CORE_BALL_H__ */
