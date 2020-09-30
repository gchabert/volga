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

#ifndef  __POINT_3D_H__
#define  __POINT_3D_H__

#include <ibex_Vector.h>
#include <math.h>
#include <cassert>

namespace volga_core {

/**
 * \brief 3D point
 */
class Point3D : public ibex::Vector {
public:
	/**
	 * \brief Creates an uninitialized point
	 */
	Point3D();

	/**
	 * \brief Creates (x,y,z).
	 */
	Point3D(double x, double y, double z);

	/**
	 * \brief Creates (list[0],list[1],list[2]).
	 */
	Point3D(std::initializer_list<double> list);

	/**
	 * \brief Assign to a vector (of size 3).
	 */
	Point3D& operator=(const ibex::Vector& v);

	/**
	 * \brief Add two points
	 */
	Point3D operator+(const Point3D& x2);

	/**
	 * \brief Subract two points
	 */
	Point3D operator-(const Point3D& x2);

	/**
	 * \brief x
	 */
	const double& x() const;

	/**
	 * \brief y
	 */
	const double& y() const;

	/**
	 * \brief z
	 */
	const double& z() const;

//private:
	explicit Point3D(const Vector& v);
};

/*
 * Non-validated distance.
 */
double distance(const Point3D& p1, const Point3D& p2);

/*============================================ inline implementation ============================================ */

inline Point3D::Point3D() : Vector({{0,0,0}}) { }

inline Point3D::Point3D(double x, double y, double z) : Vector({{x,y,z}}) {
}

inline Point3D::Point3D(std::initializer_list<double> list) : Vector(list) {
	assert(list.size()==3);
}

inline Point3D::Point3D(const Vector& v) : Vector(v) {
	assert(v.size()==3);
}

inline Point3D& Point3D::operator=(const Vector& v) {
	assert(v.size()==3);
	(Vector&) *this = v;
	return *this;
}

inline const double& Point3D::x() const { return (*this)[0]; }

inline const double& Point3D::y() const { return (*this)[1]; }

inline const double& Point3D::z() const { return (*this)[2]; }

inline double distance(const Point3D& p1, const Point3D& p2) {
	return std::sqrt(std::pow(p1.x()-p2.x(),2)+std::pow(p1.y()-p2.y(),2)+std::pow(p1.z()-p2.z(),2));
}

inline Point3D Point3D::operator+(const Point3D& x2) {
	return Point3D(((const Vector&) *this) + x2);
}
inline Point3D Point3D::operator-(const Point3D& x2) {
	return Point3D(((const Vector&) *this) - x2);
}


} /* namespace volga_core */

#endif /* __VOLGA_CORE_POINT_3D_H__ */
