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

#include <algorithm>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <sstream>
#include <map>
#include <volga_core/Mesh.h>
#include <volga_core/tf2ibex.h>
#include <volga_core/UndirectedGraph.h>

#define OM_STATIC_BUILD

#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/LongestEdgeT.hh>

using namespace std;
using namespace ibex;

namespace volga_core {

Mesh::Mesh() : root(NULL), STL_filename(""), meters(true), log(false), _max_level(0) {
}

void Mesh::init(unsigned int vertex_count, double* vertices,  unsigned int triangle_count, unsigned int* triangles,
		bool meters, double eps_meters, bool log) {
	this->log = log;
	this->meters = meters;

	if (log) cout << "Building mesh..." << endl;

	// generate vertices
	RawMesh::VertexHandle *vhandle = new RawMesh::VertexHandle[vertex_count];

	for (uint i=0; i<vertex_count; i++) {
		vhandle[i] = raw_mesh.add_vertex(RawMesh::Point(vertices[3*i],vertices[3*i+1],vertices[3*i+2]));
	}

	// generate faces
	std::vector<RawMesh::VertexHandle>  face_vhandles;

	for (uint i=0; i<triangle_count; i++) {
		face_vhandles.clear();
		face_vhandles.push_back(vhandle[triangles[3*i]]);
		face_vhandles.push_back(vhandle[triangles[3*i+1]]);
		face_vhandles.push_back(vhandle[triangles[3*i+2]]);
		raw_mesh.add_face(face_vhandles);
	}
	delete[] vhandle;

	if (log) cout << "  Created " << raw_mesh.n_faces() << " facets\n";

	init(meters, eps_meters, log);
}

void Mesh::init(const string& STL_filename, bool meters, double eps_meters, bool log) {
	this->log = log;
	this->STL_filename = STL_filename;
	this->meters = meters;

	if (log) cout << "Building mesh..." << endl;
	if (log) cout << "  STL file: " << STL_filename << endl;

	if (!OpenMesh::IO::read_mesh(raw_mesh, STL_filename)) {
		std::cerr << "Cannot read STL file\n";
		exit(1);
	}

	if (log) cout << "  Loaded " << raw_mesh.n_faces() << " facets\n";

	init(meters, eps_meters, log);
}

void Mesh::init(bool meters, double eps_meters, bool log) {

	refine_mesh(raw_mesh, eps_meters);

	load_mesh(raw_mesh);

	list<Node*> nodes; // The initial list of nodes (copy of facets)

	int num_id = 0;
	// copy the list of facets
	for (vector<Triangle>::const_iterator it=facets.begin(); it!=facets.end(); ++it) {
		nodes.push_back(new TriangleLeaf(*it));
		nodes.back()->num_id = num_id++;
	}

	//merge_nodes_slow(nodes, num_id);
	merge_facets_fast(nodes, num_id);

	set_string_ids();

	if (log) cout << "Done." << endl << endl;
}

void Mesh::init(const Ball& ball) {
	this->log = false;
	this->STL_filename = "";
	this->meters = true;
	root = new Leaf(ball);
	root->num_id = 0;
	root->id = "1";
	root->level = 0;
	_max_level = 0;
	balls.insert(make_pair(0,root));
}

void Mesh::refine_mesh(RawMesh& raw_mesh, double eps_meters) {
	double epsilon = meters? eps_meters : eps_meters*1000;

	// Initialize subdivider
	OpenMesh::Subdivider::Uniform::LongestEdgeT<RawMesh> catmull;

	// Execute subdivision steps
	//
	// If the edge has a length <= epsilon then
	// the distance between the triangle and any point in the smallest
	// enclosing circle is at most epsilon/2.
	// This way, when two balls intersect, the distance between the
	// two triangles is less than epsilon, the prescribed precision.
	catmull.set_max_edge_length(epsilon);
	catmull.attach(raw_mesh);
	catmull(1);
	catmull.detach();

	if (log) cout << "  Mesh subdivided into " << raw_mesh.n_faces() << " facets\n";
}

void Mesh::load_mesh(const RawMesh& raw_mesh) {

	double max_ball_radius=0;

	double coeff=meters? 1.0 : 0.001;

	array<array<float,3>,3> c;

	for (RawMesh::FaceIter f_it=raw_mesh.faces_begin(); f_it!=raw_mesh.faces_end(); ++f_it) {
		OpenMesh::PolyConnectivity::ConstFaceVertexRange v=f_it->vertices();
		using vertex_iterator=OpenMesh::PolyConnectivity::ConstFaceVertexRange::const_iterator;

		int i=0;
		for (vertex_iterator v_it = v.begin(); v_it!=v.end(); ++v_it) {
			const RawMesh::Point& p=raw_mesh.point(*v_it);
			int j=0;
			for (RawMesh::Point::const_iterator x_it=p.cbegin(); x_it!=p.cend(); ++x_it) {
				c[i][j++]= *x_it;
			}
			i++;
		}

		Triangle triangle(
				{{coeff*c[0][0],coeff*c[0][1],coeff*c[0][2]}},
				{{coeff*c[1][0],coeff*c[1][1],coeff*c[1][2]}},
				{{coeff*c[2][0],coeff*c[2][1],coeff*c[2][2]}}
		);

		facets.push_back(triangle);

		max_ball_radius = std::max(max_ball_radius, facets.back().enclosing_ball.radius);
	}

	if (log) cout << "  Maximal atomic ball radius: " << max_ball_radius << endl;
}

void Mesh::merge_nodes_slow(list<Node*>& nodes, int num_id) {
	if (log) cout << "  Merging facets..." << endl;

	if (facets.empty()) return;

	using iterator=list<Node*>::iterator;

	// sort facets by increasing ball volume
	nodes.sort([](const Node* x, const Node* y) { return x->ball.volume < y->ball.volume; } );

	// while not all facets have be processed
	while (nodes.size()>1) {
		// Pop the node with minimal volume
		Node* node1 = nodes.front();
		// remove it from the list
		nodes.pop_front();
		// find the iterator to merge with
		iterator it_node2 = min_element(nodes.begin(), nodes.end(),
				[node1](Node* i, Node* j) {
					return Ball::delta_volume(node1->ball,i->ball) < Ball::delta_volume(node1->ball,j->ball);
				});
		// Keep the corresponding node
		Node* node2 = *it_node2;
		// remove it also from the list
		nodes.erase(it_node2);

		// look for all other nodes that are inside the enclosing ball
		// "true": we need now a validated enclosure
		Ball b(node1->ball,node2->ball, true);
		vector<Mesh::Node*> insiders;
		insiders.push_back(node1);
		insiders.push_back(node2);

		for (iterator it=nodes.begin(); it!=nodes.end(); ++it) {
			while (it!=nodes.end() && (*it)->ball.NV_is_subset(b)) {
				insiders.push_back(*it);
				it = nodes.erase(it);
			}
		}

		// create the fusion of all these nodes
		Fusion* f = new Fusion(b, insiders);
		f->num_id = num_id++;

		// insert it in the list at the right place
		iterator it_f = find_if(nodes.begin(), nodes.end(),
				[f](Node* i) {
					return i->ball.volume >= f->ball.volume;
				});

		nodes.insert(it_f,f);
	}

	root = nodes.front();
}

void Mesh::merge_facets_fast(list<Node*>& nodes, int num_id) {
	if (log) cout << "  Merging facets..." << endl;

	if (nodes.empty()) return;

	using MyGraph = UndirectedGraph<list<Node*>::iterator>;
	MyGraph graph;

	for (list<Node*>::iterator it=nodes.begin(); it!=nodes.end(); ++it) {
		graph.add_vertex((*it)->num_id, it);
	}

	if (log) cout << "  Building neighborhood structure..." << endl;

	// Fill neighborhood structure

	// note: we assume that nodes is the initial list of facets
	// (contrary to merge_nodes_slow which will be called
	// at this end of this function with merged nodes).
	for (RawMesh::FaceIter f_it=raw_mesh.faces_begin(); f_it!=raw_mesh.faces_end(); ++f_it) {
		OpenMesh::PolyConnectivity::ConstFaceFaceRange adj=f_it->faces();
		using face_iterator=OpenMesh::PolyConnectivity::ConstFaceFaceRange::const_iterator;
		for (face_iterator ff_it = adj.begin(); ff_it!=adj.end(); ++ff_it) {
			graph.add_edge(f_it->idx(), ff_it->idx());
		}
	}

	//	for (map<long,list<Node*>::iterator>::iterator it=_map.begin(); it!=_map.end(); ++it) {
	//		MyMesh::FaceHandle handle(it->first);
	//		MyMesh::ConstFaceFaceRange adj_faces = mesh.ff_range(handle);
	//
	//		MyVertex v1(*it->second, it->first);
	//		for (MyMesh::ConstFaceFaceRange::iterator f_it=adj_faces.begin(); f_it!=adj_faces.end(); ++f_it) {
	//			list<Node*>::iterator it2=_map.find(f_it->idx())->second;
	//			MyVertex v2(*it2,f_it->idx());
	//			graph.add_edge(v1,v2);
	//		}
	//	}

	// Unfortunately some STL files are not clean
	// and result in a disconnected mesh (this can
	// be checked using g.connected_components(...)).
	// The following list stores the nodes that have
	// no more neighbor
	list<Node*> components;

	assert(num_id == raw_mesh.n_faces());

	// sort facets by increasing ball volume
	nodes.sort([](const Node* x, const Node* y) { return x->ball.volume < y->ball.volume; } );

	// while not all facets have be processed
	while (!nodes.empty()) {
		// Pop the node with minimal volume
		Node* node1 = nodes.front();

		// Get the neighbors
		MyGraph::Vertex& gnode1 = graph.find(node1->num_id);
		list<MyGraph::Vertex*>& adj1 = gnode1.adj;

		// ok store it for later and just skip it.
		// This includes the final top-level node
		// in case of a well-connected mesh.
		if (adj1.empty()) {
			components.push_back(node1);
			nodes.pop_front();
			continue;
		}

		// find the node to merge with (among the neighbors)
		MyGraph::Vertex& gnode2 = **min_element(adj1.begin(), adj1.end(),
				[&node1](const MyGraph::Vertex* n1, const MyGraph::Vertex* n2) {
			return  Ball::delta_volume(node1->ball, (*n1->data)->ball) <
					Ball::delta_volume(node1->ball, (*n2->data)->ball);
		});

		list<MyGraph::Vertex*>& adj2 = gnode2.adj;

		// Keep the corresponding node
		list<Node*>::iterator it_node2=gnode2.data;
		Node* node2 = *it_node2;

		// look for all other nodes (in the adjacency
		// of node 1 and node 2 only)
		// that are inside the enclosing ball.
		// "true": we need now a validated enclosure
		Ball b(node1->ball,node2->ball, true);
		list<MyGraph::Vertex*> ginsiders;
		vector<Node*> insiders;

		// <optimization>   This block is an optimization and can be commented out
		for (list<MyGraph::Vertex*>::iterator it=adj1.begin(); it!=adj1.end(); ++it) {
			MyGraph::Vertex& gnode3=**it;
			if (&gnode3!=&gnode2 && (*gnode3.data)->ball.NV_is_subset(b)) {
				// note: no harm if some nodes appear twice
				insiders.push_back(*gnode3.data);
				ginsiders.push_back(&gnode3);
			}
		}

		// ginsiders can accept duplicate elements
		// but not insiders. So we have to avoid duplicates.
		list<MyGraph::Vertex*>::iterator it1=ginsiders.begin();
		for (list<MyGraph::Vertex*>::iterator it2=adj2.begin(); it2!=adj2.end(); ++it2) {
			MyGraph::Vertex& gnode3=**it2;
			// insert it in the list at the right place
			if (&gnode3!=&gnode1 && (*gnode3.data)->ball.NV_is_subset(b)) {
				 it1 = find_if(it1, ginsiders.end(),
						 [&gnode3](MyGraph::Vertex* n) { return n->id>=gnode3.id; });

				if (it1==ginsiders.end() || (*it1)->id > gnode3.id) { // otherwise: already present
					insiders.push_back(*gnode3.data);
					ginsiders.insert(it1,&gnode3);
				}
			}
		}
		// </optimization>

		insiders.push_back(node1);
		ginsiders.push_back(&gnode1);
		insiders.push_back(node2);
		ginsiders.push_back(&gnode2);

		// erase all merged nodes from the list
		for (list<MyGraph::Vertex*>::iterator it=ginsiders.begin(); it!=ginsiders.end(); ++it) {
			nodes.erase((*it)->data);
		}

		// create the fusion of all these nodes
		Fusion* f = new Fusion(b, insiders);
		f->num_id = num_id;
		// insert it at the end of the list
		nodes.push_back(f);
		list<Node*>::iterator f_it=nodes.end();
		std::advance(f_it,-1);

		graph.merge_vertices(ginsiders, num_id, f_it);

		num_id++;
	}

	if (components.size()>1) // there are some disconnected nodes
		merge_nodes_slow(components, num_id);

	assert(components.size()==1);

	root = components.front();
}

void Mesh::set_string_ids() {

	if (facets.empty()) return;

	stack<Node*> s;
	root->id = "1";
	root->level = 0;
	s.push(root);
	balls.insert(make_pair(0,root));
	long i=1;
	while (!s.empty()) {
		Node& node = *s.top();
		if (node.level>_max_level) _max_level = node.level;
		s.pop();
		if (!node.leaf()) {
			Fusion& f=(Fusion&) node;
			int j=1;
			for (vector<Node*>::iterator it=f.children.begin(); it!=f.children.end(); ++it) {
				stringstream ss;
				ss <<  f.id  << (j++);
				(*it)->id = ss.str();
				(*it)->level = f.level+1;
				balls.insert(make_pair(i,*it));
				i++;
				s.push(*it);
			}
		}
	}
	if (log) cout << "  Maximal level of recursion: " << max_level() << endl;
}

void Mesh::move(const tf::Transform& t) {
	// note: raw_mesh is not updated.
	Vector transl=_ibex(t.getOrigin());
	Matrix rot=_ibex(t.getBasis());
	for (std::vector<Triangle>::iterator it=facets.begin(); it!=facets.end(); ++it) {
		for (int i=0; i<3; i++)
			it->vertices[i] = transl + rot*it->vertices[i];
		it->enclosing_ball.center = transl + rot*it->enclosing_ball.center;
	}

	for (std::map<long,Node*>::iterator it=balls.begin(); it!=balls.end(); ++it) {
		it->second->ball.center = transl + rot*it->second->ball.center;
	}
}

visualization_msgs::MarkerArray Mesh::markers_balls_level(int first_id, int level, const string& mesh_frame_name, double lifetime) const {
	visualization_msgs::MarkerArray array;

	MarkerQuery q;
	q.id = first_id;
	q.mesh_frame_name = mesh_frame_name;
	q.lifetime = lifetime;
	q.array = &array;

	root->fill_marker_array(q, level);

	return array;
}

visualization_msgs::MarkerArray Mesh::markers_active_balls(int first_id, double t, const std::string& mesh_frame_name, double lifetime) const {
	visualization_msgs::MarkerArray array;

	MarkerQuery q;
	q.id = first_id;
	q.mesh_frame_name = mesh_frame_name;
	q.lifetime = lifetime;
	q.array = &array;

	long next_pop=-1;
	try { next_pop = next_pop_after(t); }
	catch (Node::NoEvent&) { }

	root->fill_marker_array(q, t, next_pop);

	return array;
}


visualization_msgs::MarkerArray Mesh::markers_leaves(int first_id, const std::string& mesh_frame_name, double lifetime) const {
	visualization_msgs::MarkerArray array;

	MarkerQuery q;
	q.id = first_id;
	q.mesh_frame_name = mesh_frame_name;
	q.lifetime = lifetime;
	q.array = &array;

	root->fill_marker_array(q);

	return array;
}


visualization_msgs::Marker Mesh::markers_STL(int id, const string& mesh_frame_name, double lifetime) const {

	if (STL_filename=="") {
		ROS_ERROR("markers_STL called on a mesh without STL.");
		exit(0);
	}

	visualization_msgs::Marker marker;
	marker.header.frame_id = mesh_frame_name;
	marker.ns = "volga";
	marker.id = id;

	// Set the marker type.
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	//marker.mesh_resource = "package://iiwa_description/meshes/iiwa14/visual/link_0.stl";
	marker.mesh_resource = "file://"+STL_filename;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.
	// This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker
	marker.scale.x = meters? 1.0 : 1e-3;
	marker.scale.y = meters? 1.0 : 1e-3;
	marker.scale.z = meters? 1.0 : 1e-3;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.7;
	marker.color.g = 0.7;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration(lifetime);
	return marker;
}

visualization_msgs::Marker Mesh::markers_facets(int id, const string& marker_frame_id, double lifetime) const {
	visualization_msgs::Marker marker;
	marker.header.frame_id = marker_frame_id;
	marker.ns = "volga";
	marker.id = id;

	// Set the marker type.
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	geometry_msgs::Point p;
	std_msgs::ColorRGBA c;
	c.r = 1.0;
	c.g = 1.0;
	c.b = 1.0;
	c.a = 1.0;
	for (vector<Triangle>::const_iterator it=facets.begin(); it<facets.end(); ++it) {
		for (int i=0; i<3; i++) {
			p.x = it->vertices[i].x(); p.y = it->vertices[i].y(); p.z = it->vertices[i].z();
			marker.points.push_back(p);
			marker.colors.push_back(c);
		}
	}

	// Set the scale of the marker
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!

	marker.lifetime = ros::Duration(lifetime);
	return marker;
}

visualization_msgs::Marker Mesh::marker_ball(int marker_id, long ball_id, const std::string& mesh_frame_name, double lifetime) const {
	return balls.find(ball_id)->second->ball.marker(marker_id, mesh_frame_name, Ball::COLLISION, lifetime);
}

bool Mesh::Node::is_popped(double t) const {
	try {
		return ((Mesh::Node*) this)->event_before(t)->second>0;
	} catch(NoEvent&) {
		return false;
	}
}

bool Mesh::Node::add_intersection(double tstart, double tend) {
	// next event after tstart
	list<pair<double, uint> >::iterator it;
	// number of intersections before tstart
	uint n;
	bool first_intersection;

	try {
		it = event_before(tstart);
		n = it->second;
		++it;
	} catch(NoEvent&) {
		it = event_times.begin(); // can also be even_times.end()
		n = 0;
	}

	first_intersection = n==0;
	n++;
	event_times.insert(it,make_pair(tstart, n));

	if (tend==1) return first_intersection;

	while (it!=event_times.end() && it->second <= tend) {
		// same notice as above.
		n = ++it->second;
		++it;
	}

	// now, n is the number of intersections before tend
	n--;
	event_times.insert(it, make_pair(tend, n));

	// *** integrity check ***
//	n=0;
//	for (it = event_times.begin(); it!=event_times.end(); ++it) {
//		if (it->second!=n-1 && it->second!=n+1) {
//			cerr << "the event_times list is incorrect\n:[ ";
//			for (it = event_times.begin(); it!=event_times.end(); ++it) {
//				cerr << "(" << it->first << "," << it->second << ") ";
//			}
//			cerr << "]\n";
//			cerr << "tstart=" << tstart << " tend=" << tend << endl;
//			exit(0);
//		}
//		n=it->second;
//	}

	return first_intersection;
}

void Mesh::Node::add_collision(double t) {
	// a collision is necessarily the unique event
	assert(event_times.empty());
	event_times.push_back(make_pair(t, UINT_MAX));
}

bool Mesh::Node::in_collision(double t) const {
	// the collision event is necessarily the last one.
	return !event_times.empty() && event_times.back().first == t && event_times.back().second==UINT_MAX;
}

std::list<std::pair<double, uint> >::iterator Mesh::Node::event_before(double t) {
	list<pair<double, uint> >::iterator it=event_times.begin();
	while (it!=event_times.end() && t>=it->first) {
		++it;
	}
	if (it==event_times.begin()) throw NoEvent();
	else {
		std::advance(it,-1);
		return it;
	}
}

std::pair<double, uint> Mesh::Node::event_after(double t) const {
	list<pair<double, uint> >::const_iterator it=event_times.begin();
	while (it!=event_times.end() && t>=it->first) {
		++it;
	}
	if (it==event_times.end()) throw NoEvent();
	else return *it;
}

long Mesh::next_pop_after(double t) const {
	list<pair<double, long> >::const_iterator it=pops.begin();
	while (it!=pops.end() && t>it->first) {
		++it;
	}
	if (it==pops.end()) throw Node::NoEvent();
	else return it->second;
}

void Mesh::cleanup() {
	root->cleanup();
}

void Mesh::Fusion::cleanup() {
	if (!event_times.empty()) {
		for (vector<Node*>::const_iterator it=children.begin(); it!=children.end(); ++it) {
			(*it)->cleanup();
		}
	}
	event_times.clear();
}

void Mesh::Leaf::cleanup() {
	event_times.clear();
}

void Mesh::Fusion::fill_marker_array(Mesh::MarkerQuery& query) const {
	for (vector<Node*>::const_iterator it=children.begin(); it!=children.end(); ++it) {
		(*it)->fill_marker_array(query);
	}
}

void Mesh::Fusion::fill_marker_array(Mesh::MarkerQuery& query, int level) const {
	if (level==0)
		query.array->markers.push_back(
				ball.marker(
						query.id++,
						query.mesh_frame_name,
						Ball::FREE,
						query.lifetime));
	else {
		for (vector<Node*>::const_iterator it=children.begin(); it!=children.end(); ++it) {
			(*it)->fill_marker_array(query, level-1);
		}
	}
}

void Mesh::Fusion::fill_marker_array(Mesh::MarkerQuery& query, double t, long next_pop) const {

	// the following case happens only when the precision of collision set to
	// the planner is larger than that of the mesh:
	if (in_collision(t)) {
		query.array->markers.push_back(
				ball.marker(
						query.id++,
						query.mesh_frame_name,
						Ball::COLLISION,
						query.lifetime));
	} else if (is_popped(t)) { // the ball is not active (sub-balls are)
		for (vector<Node*>::const_iterator it=children.begin(); it!=children.end(); ++it) {
			// next instruction will increment query.id (and we don't know how many times)
			(*it)->fill_marker_array(query, t, next_pop);
		}
	} else {
		query.array->markers.push_back(
				ball.marker(
						query.id++,
						query.mesh_frame_name,
						num_id==next_pop? Ball::NEXT_POPPED : Ball::FREE,
								query.lifetime));
	}
}

void Mesh::Leaf::fill_marker_array(Mesh::MarkerQuery& query) const {
	query.array->markers.push_back(
			ball.marker(
					query.id++,
					query.mesh_frame_name,
					Ball::FREE,
					query.lifetime));
}

void Mesh::Leaf::fill_marker_array(Mesh::MarkerQuery& query, int level) const {
	query.array->markers.push_back(
			ball.marker(
					query.id++,
					query.mesh_frame_name,
					Ball::FREE,
					query.lifetime));
}

void Mesh::Leaf::fill_marker_array(Mesh::MarkerQuery& query, double t, long next_pop) const {
	query.array->markers.push_back(
			ball.marker(
					query.id++,
					query.mesh_frame_name,
					in_collision(t)? Ball::COLLISION : Ball::FREE,
					query.lifetime));
}

ostream& operator<<(ostream& os, const Mesh& mesh) {
	os << "Triangles: " << endl;
	for (std::vector<Triangle>::const_iterator it=mesh.facets.begin(); it!=mesh.facets.end(); ++it) {
		os << "  " << it->vertices[0] << "\t" << it->vertices[1] << '\t' << it->vertices[2] << endl;
	}
	return os << endl;
}

} // end namespace volga_core
