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

#include <volga_core/tf2ibex.h>

using namespace std;
using namespace ibex;

namespace volga_core {

ostream& operator<<(ostream& os, const tf::Transform& t) {
	os << "\n   origin=[" << t.getOrigin().x() << ' ';
	os << t.getOrigin().y() << ' ';
	os << t.getOrigin().z() << "]" << endl;
	tf::Matrix3x3 m(t.getRotation());
	os << "\n   R=(";
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			os << m[i][j] << ' ';
		os << ")" << endl;
		if (i<2)
		  os << "     (";
	}
	return os << "\n";
}

} /* namespace volga_core */
