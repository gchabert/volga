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

#include <ibex_ExprCopy.h>
#include <volga_core/FKInterpol.h>
#include <volga_core/Rotation.h>
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
FKInterpol::FKInterpol(const Function& fk, bool log) : dq0(NULL), dq1(NULL),
		dlink_ball_origin(Dim::col_vec(3)),
		dlink_ball_radius(Dim::scalar()),
		dfixed_ball_origin(Dim::col_vec(3)),
		dfixed_ball_radius(Dim::scalar()) {

	// ======== build interpolation functions ==================
	dq0 = new Domain(Dim::col_vec(fk.nb_var()));
	dq1 = new Domain(Dim::col_vec(fk.nb_var()));

	const ExprConstant& q0=ExprConstant::new_mutable(*dq0);
	const ExprConstant& q1=ExprConstant::new_mutable(*dq1);
	const ExprSymbol& t=ExprSymbol::new_("t");
	const ExprNode& q=q0 + t*(q1-q0);
	const ExprNode& fk_q=fk(q);
	fk_lerp.init(t, fk_q, (string(fk.name)+"_lerp").c_str());
	if (log) cout << "  " << fk_lerp.name << " built\n";
	if (fk.expr().type_id()==ExprNode::NumExprConstant)
		cleanup(q,false);

	SystemFactory fac;
	Array<const ExprSymbol> _t(ExprSymbol::new_("t"));
	fac.add_var(_t,IntervalVector(1,Interval(0,1)));
	const ExprNode& fk_q_copy   = ExprCopy().copy(t,_t,fk_q);
	Frame link_frame(fk_q_copy);

	const ExprConstant& link_ball_origin=ExprConstant::new_mutable(dlink_ball_origin);
	const ExprConstant& link_ball_radius=ExprConstant::new_mutable(dlink_ball_radius);
	const ExprConstant& fixed_ball_origin=ExprConstant::new_mutable(dfixed_ball_origin);
	const ExprConstant& fixed_ball_radius=ExprConstant::new_mutable(dfixed_ball_radius);
	const ExprNode& xqt=link_frame.move(link_ball_origin);
	const ExprCtr& ctr(sqrt(sqdist(xqt,fixed_ball_origin)) = link_ball_radius + fixed_ball_radius);
	fac.add_ctr(ctr);
	fk_collision = new System(fac);
	fk_solver = new DefaultSolver(*fk_collision);
	// we add _t[0] in the case of a constant expression ctr.e
	cleanup(Array<const ExprNode>(ctr.e,_t[0]),true);
	delete &ctr;

	if (log) cout << "  collision system built\n";
}

FKInterpol::~FKInterpol() {
	delete dq0;
	delete dq1;
	delete fk_collision;
	delete fk_solver;
}

void FKInterpol::set_q0(const Vector& q0) {
	dq0->v() = q0;
}

void FKInterpol::set_q1(const Vector& q1) {
	dq1->v() = q1;
}

Function& FKInterpol::get() {
	return fk_lerp;
}

bool FKInterpol::check_collision(double t) const {
	return (fk_collision->ctrs[0].f.eval(IntervalVector(1,t)).lb()<0);
}

Solver& FKInterpol::get_collision_solver() {
	return *fk_solver;
}

void FKInterpol::set_link_ball(const Ball& ball) {
	dlink_ball_origin.v() = ball.center;
	dlink_ball_radius.i() = ball.radius;
}

void FKInterpol::set_fixed_ball(const Ball& ball) {
	dfixed_ball_origin.v() = ball.center;
	dfixed_ball_radius.i() = ball.radius;
}

} // end namespace volga_core

