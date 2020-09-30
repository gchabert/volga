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

#include <volga_core/FrameInterpol.h>
#include <volga_core/tf2ibex.h>
#include <ros/package.h>
#include <list>

using namespace std;
using namespace volga_core;
using namespace ibex;


int main() {
	tf::Transform t1(tf::Quaternion::getIdentity(),tf::Vector3(0,0,0));
	tf::Transform t2(tf::Quaternion(std::sqrt(4),std::sqrt(4),std::sqrt(4),std::sqrt(4)),tf::Vector3(1,-1,2));

	FrameInterpol::set_t0_t1(t1,t2);

	Function& f=FrameInterpol::interpol();

	cout << "  tf finds:" <<
			tf::Transform(t1.getRotation().slerp(t2.getRotation(),0.5),
					t1.getOrigin().lerp(t2.getOrigin(),0.5)) << endl;

	cout << "  ibex finds:"<<
			_tf_transform(f.eval_vector(IntervalVector(1,0.5)).mid()) << endl;
}
