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

#include <volga_core/UndirectedGraph.h>
#include <list>


using namespace std;
using namespace volga_core;

int main() {
	UndirectedGraph<double> g;
	UndirectedGraph<double>::Vertex& v1=g.add_vertex(1,1);
	UndirectedGraph<double>::Vertex& v2=g.add_vertex(2,2);
	UndirectedGraph<double>::Vertex& v3=g.add_vertex(3,3);
	UndirectedGraph<double>::Vertex& v4=g.add_vertex(4,4);
	UndirectedGraph<double>::Vertex& v5=g.add_vertex(5,5);
	g.add_edge(1,2);
	g.add_edge(1,3);
	g.add_edge(2,4);
	g.add_edge(2,5);
	cout << g << endl;

	std::list<std::set<long> > l;
	g.connected_components(l);
	cout << "components:\n";
	for (set<long>& s : l) {
		cout << '{';
		for (long n : s) {
			cout << n << ' ';
		}
		cout << '}' << endl;
	}

	list<UndirectedGraph<double>::Vertex*> v;
	v.push_back(&v2);
	v.push_back(&v3);
	UndirectedGraph<double>::Vertex& v6=g.merge_vertices(v,6,6);
	cout << g << endl;

	v.clear();
	v.push_back(&v4);
	v.push_back(&v5);
	UndirectedGraph<double>::Vertex& v7=g.merge_vertices(v,7,7);
	cout << g << endl;

	v.clear();
	v.push_back(&v1);
	v.push_back(&v7);
	UndirectedGraph<double>::Vertex& v8=g.merge_vertices(v,8,8);
	cout << g << endl;

	v.clear();
	v.push_back(&v6);
	v.push_back(&v8);
	UndirectedGraph<double>::Vertex& v9=g.merge_vertices(v,9,9);
	cout << g << endl;

}
