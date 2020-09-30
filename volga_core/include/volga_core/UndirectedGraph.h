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
#ifndef __VOLGA_CORE_UNDIRECTED_GRAPH_H__
#define __VOLGA_CORE_UNDIRECTED_GRAPH_H__

#include <map>
#include <list>
#include <iostream>
#include <algorithm>
#include <utility>
#include <ros/console.h>
#include <set>

namespace volga_core {

/**
 * \brief Undirected graph
 *
 * This is a simple home-made implementation of undirected graph
 * for faster facet merging algorithm (that is, not in O(n^2)).
 * Existing open-source libraries (like the Boost Graph Library) does
 * not provide such functionality apparently because there is no
 * generic behavior to handle the "corner cases"
 */
template<typename VertexData>
class UndirectedGraph {
	public:
	/**
	 * \brief Vertex
	 */
	class Vertex {
		public:

		/**
		 * \brief Creates a new vertex with associated data
		 */
		Vertex(long id, const VertexData& data) : id(id), data(data) { }

		/**
		 * \brief Add an edge (set vertex as a neighbor)
		 */
		void add_neighbor(Vertex& vertex) {
			typename std::list<Vertex*>::iterator it = find_if(adj.begin(), adj.end(), [&vertex](const Vertex* n) { return n->id >= vertex.id; });
			if (it==adj.end() || (*it)->id>vertex.id) adj.insert(it,&vertex); // otherwise: vertex is already in.
		}

		/**
		 * \brief Remove an edge (remove a vertex from neighbors)
		 */
		void rem_neighbor(Vertex& vertex) {
			typename std::list<Vertex*>::iterator it = std::find_if(adj.begin(), adj.end(), [&vertex](const Vertex* n) { return n->id==vertex.id; });
			if (it!=adj.end()) adj.erase(it);
		}

		/**
		 * \brief Vertex id
		 */
		long id;
		/**
		 * \brief Vertex data
		 */
		VertexData data;

		/*
		 * Adjacency
		 *
		 * Sorted list (by id) of adjacent vertices
		 */
		std::list<Vertex*> adj;
	};


	/**
	 * \brief Add a vertex in the graph
	 */
	Vertex& add_vertex(long id, const VertexData& data) {
		if (_map.find(id)!=_map.end()) {
			ROS_ERROR_STREAM("[UndirectedGraph] Element " << id << " already exists.");
			exit(1);
		}
		Vertex* vertex = new Vertex(id, data);
		_map.insert(std::make_pair(id, vertex));
		return *vertex;
	}

	/**
	 * \brief Find a vertex
	 */
	Vertex& find(long id) {
		typename std::map<long, Vertex*>::iterator it=_map.find(id);
		if (it==_map.end()) {
			ROS_ERROR_STREAM("[UndirectedGraph] Element " << id << " not found.");
			exit(1);
		}
		return *it->second;
	}

	/**
	 * \brief Add an edge between two vertices
	 */
	void add_edge(long id1, long id2) {
		Vertex& vertex1 = find(id1);
		Vertex& vertex2 = find(id2);
		vertex1.add_neighbor(vertex2);
		vertex2.add_neighbor(vertex1);
	}

	/**
	 * \brief Merge all vertices in a (sorted?) list
	 *
	 * \param v        - the list of vertices
	 * \param new_id   - the id of the resulting vertex
	 * \param new_data - the data associated to the resulting vertex
	 */
	Vertex& merge_vertices(const std::list<Vertex*>& v, long new_id, const VertexData& new_data) {
		assert(v.size()>=2);

		Vertex& new_vertex = add_vertex(new_id, new_data);

		// make the adj(new_vertex), the adjacency of new_vertex to be the union: adj(vertex1) U adj(vertex2) U ...
		for (typename std::list<Vertex*>::const_iterator it=v.begin(); it!=v.end(); ++it) {
			new_vertex.adj.merge((*it)->adj, [](const Vertex* v1, const Vertex* v2) { return v1->id<v2->id;}); // note: now vertex.adj is empty.
		}

		// removes common adjacent vertices between vertex1, vertex2, ..
		typename std::list<Vertex*>::iterator last = std::unique(new_vertex.adj.begin(), new_vertex.adj.end(), [](const Vertex* v1, const Vertex* v2) { return v1->id==v2->id;});
		new_vertex.adj.erase(last, new_vertex.adj.end());

		assert(!new_vertex.adj.empty());

		// removes old vertices from adjacency of new_vertex
		// and erase vertices
		for (typename std::list<Vertex*>::const_iterator it=v.begin(); it!=v.end(); ++it) {
			new_vertex.rem_neighbor(**it);
		}

		// removes old vertices from the adjacency of the (remaining) adjacent vertices
		// and add the new one
		for (typename std::list<Vertex*>::iterator it = new_vertex.adj.begin(); it!=new_vertex.adj.end(); ++it) {
			Vertex& vertex=**it;
			for (typename std::list<Vertex*>::const_iterator it2=v.begin(); it2!=v.end(); ++it2) {
				vertex.rem_neighbor(**it2);
			}
			vertex.add_neighbor(new_vertex);
		}

		// erase vertices
		for (typename std::list<Vertex*>::const_iterator it=v.begin(); it!=v.end(); ++it) {
			_map.erase((*it)->id);
			delete *it;
		}

		return new_vertex;
	}

	/**
	 * \brief Connected components
	 *
	 * Return the list of all connecter components,
	 * each component being a set of vertex ids.
	 */
	void connected_components(std::list<std::set<long> >& components) {
		if (_map.empty()) return;

		// copy the map
		std::map<long, Vertex*> map_copy(_map);

		// all the remaining vertices to be processed in the current component
		std::list<Vertex*> to_process;

		while (!map_copy.empty()) {
			//std::cout << "new component\n";
			// create a new empty connected component
			components.push_back(std::set<long>());

			std::set<long>& s = components.back(); // current component

			Vertex* vertex = map_copy.begin()->second;
			s.insert(vertex->id);         // add this vertex in the component
			map_copy.erase(vertex->id);   // and forget it henceforth
			to_process.push_back(vertex); // process later its adjacent vertices

			while (!to_process.empty()) {
				vertex = to_process.front();
				//std::cout << "vertex: " << vertex->id << std::endl;
				for (typename std::list<Vertex*>::iterator it = vertex->adj.begin(); it!=vertex->adj.end(); ++it) {
					if (s.find((*it)->id)==s.end()) { // not yet present
						assert(map_copy.find((*it)->id)!=map_copy.end()); // cannot be in a previous component!
						s.insert((*it)->id);         // add this vertex in the component
						map_copy.erase((*it)->id);   // and forget it henceforth
						// note: important to add vertices in the set here, as they are discovered,
						// and not when they are popped from the to_process list because vertices
						// can get accumulated in this list and complexity turn very bad!
						to_process.push_back(*it);
					}
				}
				to_process.pop_front(); // ok: this vertex has been processed
			}
		}
	}

	std::map<long, Vertex*> _map;
};

/**
 * \brief Display the undirected graph.
 */
template<typename T>
std::ostream& operator<<(std::ostream& os, const UndirectedGraph<T>& g) {
	for (auto it = g._map.begin(); it!=g._map.end(); ++it) {
		os << it->first << " : (";
		typename UndirectedGraph<T>::Vertex* vertex=it->second;
		for (auto it2 = vertex->adj.begin(); it2!=vertex->adj.end(); ++it2) {
			if (it2!=vertex->adj.begin()) os << ',';
			os << (*it2)->id;
		}
	os << ")" << std::endl;
 	}
	return os;
}

} /* namespace volga_core */

#endif /* __VOLGA_CORE_UNDIRECTED_GRAPH_H__ */
