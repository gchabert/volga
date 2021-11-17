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

#include <ibex_SystemFactory.h>
#include <ibex_DefaultSolver.h>
#include <ibex_ExprCopy.h>
#include <volga_core/FrameInterpol.h>
#include <volga_core/tf2ibex.h>

using namespace std;
using namespace ibex;

namespace volga_core {

namespace {

const ExprNode& sqdist(const ExprNode& x, const ExprNode& y) {
	assert(x.dim.is_vector());
	assert(x.dim==y.dim);
	const ExprNode* e=&pow(x[0]-y[0],2);
	for (int i=1; i<x.dim.vec_size(); i++)
		e = &(*e + pow(x[i]-y[i],2));
	return *e;
}

}

FrameInterpol& FrameInterpol::get() {
	static FrameInterpol f;
	return f;
}

void FrameInterpol::set_t0_t1(const tf::Transform& t0, const tf::Transform& t1) {
	get()._set_t0_t1(t0,t1);
}

void FrameInterpol::set_link_ball(const Ball& ball) {
	get().dlink_ball_origin.v() = ball.center;
	get().dlink_ball_radius.i() = ball.radius;
}

void FrameInterpol::set_fixed_ball(const Ball& ball) {
	get().dfixed_ball_origin.v() = ball.center;
	get().dfixed_ball_radius.i() = ball.radius;
}

Function& FrameInterpol::interpol() {
	FrameInterpol& f=get();
	return f.is_lerp ? f.lerp : f.slerp;
}

bool FrameInterpol::check_collision(double t) {
	FrameInterpol& f=get();
	System* sys=f.is_lerp ? f.lerp_collision : f.slerp_collision;
	return sys->ctrs[0].f.eval(IntervalVector(1,t)).lb()<0;
}

Solver& FrameInterpol::get_collision_solver() {
	FrameInterpol& f=get();
	return f.is_lerp ? *f.lerp_solver : *f.slerp_solver;
}

UnitQuaternion FrameInterpol::build_q0() {
	const ExprConstant& q0x = ExprConstant::new_mutable(dq0x);
	const ExprConstant& q0y = ExprConstant::new_mutable(dq0y);
	const ExprConstant& q0z = ExprConstant::new_mutable(dq0z);
	const ExprConstant& q0w = ExprConstant::new_mutable(dq0w);
	// "false" because q0 is a validated unit quaternion
	return UnitQuaternion(q0x,q0y,q0z,q0w,false);
}

UnitQuaternion FrameInterpol::build_q1() {
	const ExprConstant& q1x = ExprConstant::new_mutable(dq1x);
	const ExprConstant& q1y = ExprConstant::new_mutable(dq1y);
	const ExprConstant& q1z = ExprConstant::new_mutable(dq1z);
	const ExprConstant& q1w = ExprConstant::new_mutable(dq1w);
	// "false" because q1 is a validated unit quaternion
	return UnitQuaternion(q1x,q1y,q1z,q1w,false);
}

const ExprNode& FrameInterpol::build_x(const ExprSymbol& t) {
	const ExprConstant& x0 = ExprConstant::new_mutable(dx0);
	const ExprConstant& x1 = ExprConstant::new_mutable(dx1);
	return x0 + t*(x1-x0);
}

FrameInterpol::FrameInterpol() :
		dx0(Dim::col_vec(3)),
		dx1(Dim::col_vec(3)),
		dq0x(Dim::scalar()),
		dq0y(Dim::scalar()),
		dq0z(Dim::scalar()),
		dq0w(Dim::scalar()),
		dq1x(Dim::scalar()),
		dq1y(Dim::scalar()),
		dq1z(Dim::scalar()),
		dq1w(Dim::scalar()),
		dqdot(Dim::scalar()),
		dtheta_0(Dim::scalar()),
		dlink_ball_origin(Dim::col_vec(3)),
		dlink_ball_radius(Dim::scalar()),
		dfixed_ball_origin(Dim::col_vec(3)),
		dfixed_ball_radius(Dim::scalar()) {


	// WARNING: if we do simplification of the expressions, the
	// reference to domains will not be valid anymore. This
	// has to be fixed in Ibex (reference constants should not
	// be removed by ExprSimplify).



	// for the rotation, we use SLERP (spherical linear interpolation)
	// even if we don't actually need a uniform interpolation.

	// There is actually 2 cases in SLERP, when q1.q2~0 and q1.q2>>0
	// so we need two expressions, one for each case.
	// Using just one expression for all cases with intervals would lead
	// to ill-conditioned problems hence drastic overestimation. E.g.,
	// using formula 1 (linear interpolation + normalization) in the case
	// where q1=-q2 would lead to ([-oo,oo],...,[-oo,oo]).

	{
		const ExprSymbol& t = ExprSymbol::new_("t");
		const ExprNode& xt  = build_x(t);
		UnitQuaternion q0   = build_q0();
		UnitQuaternion q1   = build_q1();

		UnitQuaternion qt(q0 + t*(q1-q0), true); // "true" = normalization

		const ExprNode& xqt = qt.stack_with(xt);

		lerp.init(t, xqt, "lerp");
	}

	{   // note: we cannot use ExprCopy because
		// we want constants pointing to the same domains!

		SystemFactory fac;

		const ExprSymbol& t = ExprSymbol::new_("t");
		fac.add_var(t);
		const ExprNode& xt  = build_x(t);
		UnitQuaternion q0   = build_q0();
		UnitQuaternion q1   = build_q1();

		UnitQuaternion qt(q0 + t*(q1-q0), true); // "true" = normalization
		const ExprConstant& link_ball_origin=ExprConstant::new_mutable(dlink_ball_origin);
		const ExprConstant& link_ball_radius=ExprConstant::new_mutable(dlink_ball_radius);
		const ExprConstant& fixed_ball_origin=ExprConstant::new_mutable(dfixed_ball_origin);
		const ExprConstant& fixed_ball_radius=ExprConstant::new_mutable(dfixed_ball_radius);
		const ExprNode& ft = xt + qt.rotate(link_ball_origin);

		fac.add_ctr(sqrt(sqdist(ft,fixed_ball_origin)) = link_ball_radius+fixed_ball_radius);

		lerp_collision = new System(fac);

		lerp_solver = new DefaultSolver(*lerp_collision);
	}

	{
		const ExprSymbol& t = ExprSymbol::new_("t");
		const ExprNode& xt  = build_x(t);
		UnitQuaternion q0   = build_q0();
		UnitQuaternion q1   = build_q1();

		const ExprConstant& dot=ExprConstant::new_mutable(dqdot);
		const ExprConstant& theta_0=ExprConstant::new_mutable(dtheta_0);
		const ExprNode& theta = theta_0*t;
		const ExprNode& sin_theta = sin(theta);
		const ExprNode& sin_theta_0 = sin(theta_0);

		const ExprNode& s2 = sin_theta / sin_theta_0;
		const ExprNode& s1 = cos(theta) - dot * s2;

		UnitQuaternion qt((s1 * q0) + (s2 * q1), false);

		const ExprNode& xqt = qt.stack_with(xt);

		slerp.init(t, xqt, "slerp");
	}

	{   // note: we could use now
		// expression copy (thanks to mutable constants)

		// copy for slerp_point
		//		Array<const ExprSymbol> _t(ExprSymbol::new_("t"));
		//		ExprCopy c;
		//		const ExprNode& _xt = c.copy(t,_t,xt,true);
		//		UnitQuaternion _qt(qt,t,_t); //ok: no node to be shared with _xt
		SystemFactory fac;

		const ExprSymbol& t = ExprSymbol::new_("t");
		fac.add_var(t);

		const ExprNode& xt  = build_x(t);
		UnitQuaternion q0   = build_q0();
		UnitQuaternion q1   = build_q1();

		const ExprConstant& dot=ExprConstant::new_mutable(dqdot);
		const ExprConstant& theta_0=ExprConstant::new_mutable(dtheta_0);
		const ExprNode& theta = theta_0*t;
		const ExprNode& sin_theta = sin(theta);
		const ExprNode& sin_theta_0 = sin(theta_0);

		const ExprNode& s2 = sin_theta / sin_theta_0;
		const ExprNode& s1 = cos(theta) - dot * s2;

		UnitQuaternion qt((s1 * q0) + (s2 * q1), false);
		const ExprConstant& link_ball_origin=ExprConstant::new_mutable(dlink_ball_origin);
		const ExprConstant& link_ball_radius=ExprConstant::new_mutable(dlink_ball_radius);
		const ExprConstant& fixed_ball_origin=ExprConstant::new_mutable(dfixed_ball_origin);
		const ExprConstant& fixed_ball_radius=ExprConstant::new_mutable(dfixed_ball_radius);

		const ExprNode& ft = xt + qt.rotate(link_ball_origin);

		fac.add_ctr(sqrt(sqdist(ft,fixed_ball_origin)) = link_ball_radius+fixed_ball_radius);

		slerp_collision = new System(fac);

		slerp_solver = new DefaultSolver(*slerp_collision);
	}

}

FrameInterpol::~FrameInterpol() {
	delete lerp_collision;
	delete slerp_collision;
	delete lerp_solver;
	delete slerp_solver;
}

void FrameInterpol::_set_t0_t1(const tf::Transform& t0, const tf::Transform& t1) {
	// Create validated unit quaternions from input ones
	IntervalVector q0(_ibex(t0.getRotation()));
	q0 = (Interval::one()/q0.norm2())*q0;

	IntervalVector q1(_ibex(t1.getRotation()));
	q1 = (Interval::one()/q1.norm2())*q1;

	// ****** Update all constants *****
	dx0.v() = _ibex(t0.getOrigin());
	dx1.v() = _ibex(t1.getOrigin());

	dq0x.i() = q0[0];
	dq0y.i() = q0[1];
	dq0z.i() = q0[2];
	dq0w.i() = q0[3];

	dq1x.i() = q1[0];
	dq1y.i() = q1[1];
	dq1z.i() = q1[2];
	dq1w.i() = q1[3];

	// Compute the cosine of the angle between the two vectors.
	dqdot.i() = q0*q1;
	dtheta_0.i() = acos(dqdot.i());
	// **********************************

	// <wikipedia>
	// If the dot product is negative, slerp won't take
    // the shorter path. Note that q1 and -q1 are equivalent when
    // the negation is applied to all four components. Fix by
    // reversing one quaternion.
	// </wikipedia>
    if (dqdot.i().ub() < 0) {
    	dq0x.i()  = -dq0x.i();
    	dq0y.i()  = -dq0y.i();
    	dq0z.i()  = -dq0z.i();
    	dq0w.i()  = -dq0w.i();
        dqdot.i() = -dqdot.i();
        dtheta_0.i() = Interval::pi()-dtheta_0.i();
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dqdot.i().lb() > DOT_THRESHOLD) {
    	// If the inputs are too close for comfort, linearly interpolate
    	// and normalize the result.
    	is_lerp = true;
    } else {
    	is_lerp = false;
    }
}


} /* namespace volga_core */
