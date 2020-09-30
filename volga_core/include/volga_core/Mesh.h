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

#ifndef __VOLGA_CORE_MESH_H__
#define __VOLGA_CORE_MESH_H__

#include <visualization_msgs/MarkerArray.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMeshT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <ibex.h>
#include <list>
#include <map>

#include "Ball.h"
#include "Frame.h"
#include "Triangle.h"

namespace volga_core {

/**
 * \brief Tree-structured mesh.
 *
 * The mesh is defined by a classical set of triangle
 * facets and a tree structure of balls representing
 * different levels of approximation of the geometry.
 * This allows fast collision computation.
 */
class Mesh {
public:

	/**
	 * \brief Uninitialized mesh.
	 */
	Mesh();

	/**
	 * \brief Build the mesh from a STL file.
	 *
	 * \param STL_filename - file name, must be absolute path
	 * \param meters       - if true, the dimensions are in meters,
	 *                       otherwise inmillimeters
	 * \param eps_meters   - the precision **in meters** the mesh
	 *                       must be refined with (the precision is in
	 *                       meters even if meters=false).
	 * \param log          - verbose if true
	 */
	void init(const std::string& STL_filename, bool meters, double eps_meters, bool log=false);

	/**
	 * \brief Build the mesh as a simple ball.
	 *
	 * In this case there is no facet, only the
	 * enclosing ball.
	 */
	void init(const Ball& ball);

	/**
	 * \brief Build the mesh from a list of vertices/triangles.
	 *
	 * \param vertex_count   - The number of available vertices
	 * \param vertices       - The position for each vertex. Vertex k has values at
	 *                         index (3k, 3k+1, 3k+2) = (x,y,z)
	 * \param triangle_count - The number of triangles formed with the vertices
	 * \param triangles      - The vertex indices for each triangle. Triangle k
	 *                         has vertices at index (3k, 3k+1, 3k+2) = (v1, v2, v3)
	 * \param meters         - True if the unit is meter (false=millimeter)
	 * \param eps_meters     - the precision **in meters** the mesh must be refined
	 *                         with (the precision is in meters even if meters=false).
	 * \param log            - Verbose if true
	 */
	void init(unsigned int vertex_count, double* vertices,  unsigned int triangle_count, unsigned int* triangles,  bool meters, double eps_meters, bool log=false);

	/**
	 * \brief Change the pose of the mesh in the world.
	 *
	 * The world frame is the one in which the mesh is initially built.
	 */
	void move(const tf::Transform& t);

	/**
	 * \brief Maximal level of recursion in the ball structure
	 */
	unsigned int max_level() const;

	/**
	 * \brief Marker query (for RVIZ)
	 *
	 * This class allows to specify how the mesh should be
	 * rendered under RVIZ.
	 */
	class MarkerQuery {
	public:
		/**
		 * \brief Current node id
		 *
		 * The markers created for the whole mesh are numbered
		 * starting from an initial id supplied by the caller.
		 */
		int id;

		/**
		 * \brief Mesh frame name
		 *
		 * The frame shared by all markers.
		 */
		std::string mesh_frame_name;

		/**
		 * \brief Marker lifetime
		 *
		 * Duration shared by all markers.
		 */
		double lifetime;

		/**
		 * \brief Set of markers
		 *
		 * The set of markers to be filled while traversing
		 * the tree structure.
		 */
		visualization_msgs::MarkerArray* array;
	};

	/**
	 * \brief Node in the tree structure
	 */
	class Node {
	public:
		/**
		 * \brief No event exception
		 *
		 * Thrown when no event exists for the request
		 * \see event_before(...)
		 * \see event_after(...)
		 */
		class NoEvent { };

		/**
		 * \brief Build a node with the given ball
		 */
		Node(const Ball& ball);

		/**
		 * \brief Delete this
		 */
		virtual ~Node();

		/**
		 * \brief True if this node is a leaf
		 */
		virtual bool leaf() const=0;

		/**
		 * \brief List of recorded events for the node.
		 *
		 * The list is sorted by increasing time.
		 *
		 * An event is either when the ball of this node enters or quits
		 * intersection with another node ball. The pair contains the
		 * time of that event and the total number of intersections at
		 * that time.
		 *
		 * By convention if there is a collision, the number of
		 * intersections is UINT_MAX. There is only one possible
		 * event in this case.
		 *
		 */
		std::list<std::pair<double, uint> > event_times;

		/**
		 * \brief Return the event preceding t.
		 */
		std::list<std::pair<double, uint> >::iterator event_before(double t);

		/**
		 * \brief Return next event after t.
		 */
		std::pair<double, uint> event_after(double t) const;

		/**
		 * \brief Add a time where this node enters/quits intersection with another.
		 *
		 * If the node enters intersection and if it was "free" so far, it is "popped".
		 * If the node quits intersection and the number of intersections becomes 0,
		 * the node becomes "free".
		 *
		 * If tend==1, this means that the node does not quit intersection.
		 *
		 * \return true if the new intersection is the only one at time t (pop).
		 */
		bool add_intersection(double tstart, double tend);

		/**
		 * \brief Mark the node in collision at time t.
		 */
		void add_collision(double t);

		/**
		 * \brief True if the ball of this node is popped at time t.
		 *
		 * Means that the number of intersections at t is >0.
		 *
		 * Warning: only works with t < tmax, with tmax=the result of
		 * LocalPlanner::tmax().
		 */
		bool is_popped(double t) const;

		/**
		 * \brief True if this node is in collision at time t.
		 */
		bool in_collision(double t) const;

		/**
		 * \brief Remove all events
		 */
		virtual void cleanup()=0;

		/**
		 * \brief Build markers for balls at a specific level (for RVIZ)
		 *
		 * \param query - the marker query
		 * \param level - the level in the tree where balls should appear
		 *                (the root is at level 0)
		 */
		virtual void fill_marker_array(MarkerQuery& query, int level) const=0;

		/**
		 * \brief Build markers for the mesh state at a specific time (for RVIZ)
		 *
		 * \param query    - the marker query
		 * \param t        - the time
		 * \param next_pop - id number of the next ball that will be popped at time t.
		 */
		virtual void fill_marker_array(MarkerQuery& query, double t, long next_pop) const=0;

		/**
		 * \brief Build markers for all leaves (for RVIZ)
		 *
		 * \param query    - the marker query
		 */
		virtual void fill_marker_array(MarkerQuery& query) const=0;

		/**
		 * \brief Node enclosing ball
		 */
		volga_core::Ball ball;

		/**
		 * \brief Node level (0=root)
		 */
		uint level;

		/**
		 * \brief Node identifying string
		 *
		 * For debug only.
		 */
		std::string id;

		/**
		 * \brief Node identifying number
		 */
		long num_id;
	};

	/**
	 * \brief Leaf in the tree structure
	 */
	class Leaf : public Node {
	public:

		/**
		 * \brief Build a leaf
		 */
		Leaf(const Ball& ball);

		/**
		 * \brief Return true
		 */
		virtual bool leaf() const override;

		/**
		 * \brief Remove all events
		 */
		virtual void cleanup() override;

		/**
		 * \brief Build markers for balls at a specific level (for RVIZ)
		 *
		 * \param query - the marker query
		 * \param level - the level in the tree where balls should appear
		 *                (the root is at level 0)
		 */
		virtual void fill_marker_array(MarkerQuery& query, int level) const override;

		/**
		 * \brief Build markers for the mesh state at a specific time (for RVIZ)
		 *
		 * \param query    - the marker query
		 * \param t        - the time
		 * \param next_pop - id number of the next ball that will be popped at time t.
		 */
		virtual void fill_marker_array(MarkerQuery& query, double t, long next_pop) const override;

		/**
		 * \brief Build markers for all leaves (for RVIZ)
		 *
		 * \param query    - the marker query
		 */
		virtual void fill_marker_array(MarkerQuery& query) const override;
	};

	/**
	 * \brief Leaf built from a triangle
	 */
	class TriangleLeaf : public Leaf {
	public:
		/**
		 * \brief Build a leaf from a triangle
		 */
		TriangleLeaf(const Triangle& tr);

		/*
		 * \brief The triangle.
		 */
		const Triangle& tr;
	};

	/**
	 * \brief Fusion of nodes
	 *
	 * The ball is enclosing the subnodes ball.
	 */
	class Fusion : public Node {
	public:

		/**
		 * \brief Build a fusion node
		 *
		 * \param b        - the enclosing ball
		 * \param children - subnodes
		 */
		Fusion(const Ball& b, const std::vector<Node*>& children);

		/**
		 * \brief Return false
		 */
		virtual bool leaf() const override;

		/**
		 * \brief Remove all events
		 */
		virtual void cleanup() override;

		/**
		 * \brief Build markers for balls at a specific level (for RVIZ)
		 *
		 * \param query - the marker query
		 * \param level - the level in the tree where balls should appear
		 *                (the root is at level 0)
		 */
		virtual void fill_marker_array(MarkerQuery& query, int level) const override;

		/**
		 * \brief Build markers for the mesh state at a specific time (for RVIZ)
		 *
		 * \param query    - the marker query
		 * \param t        - the time
		 * \param next_pop - id number of the next ball that will be popped at time t.
		 */
		virtual void fill_marker_array(MarkerQuery& query, double t, long next_pop) const override;

		/**
		 * \brief Build markers for all leaves (for RVIZ)
		 *
		 * \param query    - the marker query
		 */
		virtual void fill_marker_array(MarkerQuery& query) const override;

		/**
		 * \brief Child nodes
		 */
		std::vector<Node*> children;
	};


	/**
	 * \brief Return the initial mesh marker (all facets)
	 */
	visualization_msgs::Marker markers_STL(int id, const std::string& mesh_frame_name, double lifetime) const;

	/**
	 * \brief Return a ball of given id 'ball_id'.
	 */
	visualization_msgs::Marker marker_ball(int marker_id, long ball_id, const std::string& mesh_frame_name, double lifetime) const;

	/**
	 * \brief Return all balls at a given depth level
	 */
	visualization_msgs::MarkerArray markers_balls_level(int first_id, int level, const std::string& mesh_frame_name, double lifetime) const;

	/**
	 * \brief Return all active balls at time t.
	 *
	 * *** To be called after LocalPlanner::max_time() ***
	 */
	visualization_msgs::MarkerArray markers_active_balls(int first_id, double t, const std::string& mesh_frame_name, double lifetime) const;

	/**
	 * \brief Return all leaves (balls of facets)
	 */
	visualization_msgs::MarkerArray markers_leaves(int first_id, const std::string& mesh_frame_name, double lifetime) const;

	/**
	 * \brief Return all facets (equivalent to markers_STL)
	 */
	visualization_msgs::Marker markers_facets(int id, const std::string& mesh_frame_name, double lifetime) const;

	/**
	 * \brief Return the id of the next ball that will be popped
	 * from time t (t being included: if a ball pops at time t,
	 * this is the one returned).
	 */
	long next_pop_after(double t) const;

	/**
	 * \brief Remove all event information
	 */
	void cleanup();

protected:
	friend class LocalPlanner;

	typedef OpenMesh::TriMesh_ArrayKernelT<OpenMesh::DefaultTraits> RawMesh;

	friend std::ostream& operator<<(std::ostream& os, const Mesh& mesh);

	/*
	 * Initialize the mesh
	 *
	 * \see constructors
	 */
	void init(bool meters, double eps_meters, bool log);

	/*
	 * Refine the mesh.
	 *
	 * This function will subdivide the triangles until
	 * the lengths of every edges are less than eps_meters.
	 */
	void refine_mesh(RawMesh& raw_mesh, double eps_meters);

	/*
	 * Load the mesh from OpenMesh structure.
	 */
	void load_mesh(const RawMesh& raw_mesh);

	/**
	 * Merge the nodes (slow variant)
	 *
	 * Note: this function is still useful for
	 * merging the last nodes that are disconnected
	 * when using merge_facets_fast.
	 *
	 * That is why the list in argument is not
	 * necessarily the initial list of facets (Leaf nodes).
	 */
	void merge_nodes_slow(std::list<Node*>& nodes, int num_id);

	/*
	 * Merge the nodes (faster variant)
	 *
	 * \param num_id - first id of the newly created nodes.
	 */
	void merge_facets_fast(std::list<Node*>& nodes, int num_id);

	/*
	 * Set the nodes identifying strings and levels.
	 */
	void set_string_ids();

	/*
	 * STL name of the file (if any)
	 */
	std::string STL_filename;

	/*
	 * Whether the unit is meter or millimeter
	 */
	bool meters;

	/*
	 * list of pops (set by LocalPlanner)
	 */
	std::list<std::pair<double, long> > pops;

	/*
	 * Raw facets
	 */
	std::vector<Triangle> facets;

	/*
	 * Map to retrieve a node from its id
	 */
	std::map<long,Node*> balls;

	/**
	 * Root node of the mesh
	 */
	Node* root;

	/*
	 * Max level (mainly for debug)
	 */
	unsigned int _max_level;

	/*
	 * Verbose if true
	 */
	bool log;

	/*
	 * The OpenMesh structure
	 */
	RawMesh raw_mesh;

};

/**
 * \brief Display a mesh
 */
std::ostream& operator<<(std::ostream& os, const Mesh& mesh);

/*============================================ inline implementation ============================================ */


inline Mesh::Node::Node(const Ball& ball) : ball(ball), level(0), num_id(-1) { }

inline Mesh::Node::~Node() { }

inline Mesh::Leaf::Leaf(const Ball& ball) : Node(ball) { }

inline Mesh::TriangleLeaf::TriangleLeaf(const Triangle& tr) : Leaf(tr.enclosing_ball), tr(tr) { }

inline bool Mesh::Leaf::leaf() const { return true; }

inline Mesh::Fusion::Fusion(const Ball& b, const std::vector<Node*>& children) :
		Node(b), children(children) { }

inline bool Mesh::Fusion::leaf() const { return false; }

inline unsigned int Mesh::max_level() const { return _max_level; }

} /* namespace volga_core */

#endif /* __VOLGA_CORE_MESH_H__ */
