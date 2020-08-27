#ifndef ADTREE_SKELETON_H
#define ADTREE_SKELETON_H

/*
*	Copyright (C) 2019 by
*       Shenglan Du (dushenglan940128@163.com)
*       Liangliang Nan (liangliang.nan@gmail.com)
*       3D Geoinformation, TU Delft, https://3d.bk.tudelft.nl
*
*	This file is part of AdTree, which implements the 3D tree
*   reconstruction method described in the following paper:
*   -------------------------------------------------------------------------------------
*       Shenglan Du, Roderik Lindenbergh, Hugo Ledoux, Jantien Stoter, and Liangliang Nan.
*       AdTree: Accurate, Detailed, and Automatic Modeling of Laser-Scanned Trees.
*       Remote Sensing. 2019, 11(18), 2074.
*   -------------------------------------------------------------------------------------
*   Please consider citing the above paper if you use the code/program (or part of it).
*
*	AdTree is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	AdTree is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <3rd_party/kd_tree/Vector3D.h>
#include <3rd_party/kd_tree/KdTree.h>
#include <easy3d/core/types.h>

namespace easy3d {
	class PointCloud;
    class SurfaceMesh;
}

//define the vertex and edge properties
struct SGraphVertexProp
{
	easy3d::vec3  cVert;
	std::size_t nParent;
	double lengthOfSubtree;

    double radius; // used only by the smoothed skeleton
    bool   visited;
};

struct SGraphEdgeProp
{
	double nWeight;
	double nRadius;
	std::vector<int> vecPoints;
};


//define the tree graph
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SGraphVertexProp, SGraphEdgeProp > Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor SGraphVertexDescriptor;
typedef boost::graph_traits<Graph>::edge_descriptor SGraphEdgeDescriptor;
typedef boost::graph_traits<Graph>::vertex_iterator SGraphVertexIterator;
typedef boost::graph_traits<Graph>::edge_iterator SGraphEdgeIterator;
typedef boost::graph_traits<Graph>::adjacency_iterator SGraphAdjacencyIterator;
typedef boost::graph_traits<Graph>::out_edge_iterator  SGraphOutEdgeIterator;


//define the tree leaves
struct Leaf
{
	easy3d::vec3 cPos; 
	easy3d::vec3 cNormal;
	easy3d::vec3 cDir;
	double nRad;
	double nLength;
	SGraphVertexDescriptor pSource;
};


//define the graph path to smooth branches
typedef std::vector<SGraphVertexDescriptor> Path;


//define the tree skeleton
class Skeleton
{
public:
	Skeleton();
	~Skeleton();

    bool reconstruct_branches(const easy3d::PointCloud* cloud, easy3d::SurfaceMesh* result);
    bool reconstruct_leaves(easy3d::SurfaceMesh* result);

    /*-------------------------------------------------------------*/
    /*------------ retrieve the intermediate results --------------*/
    /*-------------------------------------------------------------*/
    const Graph& get_delaunay() const { return delaunay_; }
    const Graph& get_mst() const { return MST_; }
    const Graph& get_simplified_skeleton() const { return simplified_skeleton_; }
    const Graph& get_smoothed_skeleton() const { return smoothed_skeleton_; }

    struct Branch {
        std::vector<easy3d::vec3> points;
        std::vector<double>       radii;
    };
    std::vector<Branch> get_branches_parameters() const;

private:

	/*-------------------------------------------------------------*/
	/*---------------------pipeline method-------------------------*/
	/*-------------------------------------------------------------*/
	//build the initial delaunay graph from input point cloud
    bool build_delaunay(const easy3d::PointCloud* cloud);

	//extract the minimum spanning tree from delaunay graph
    bool extract_mst();

    //simplify the MST
    bool simplify_skeleton();

    //determine branch radius
    bool compute_branch_radius();

    bool smooth_skeleton();

    //extract branch surfaces
    bool extract_branch_surfaces(easy3d::SurfaceMesh* result);

	//add random leaves 
    bool add_leaves();

	/*-------------------------------------------------------------*/
	/*------method for skeleton refining and simplification--------*/
	/*-------------------------------------------------------------*/
	//eliminate unimportant small edges and keep the main skeleton
    void keep_main_skeleton(Graph* i_Graph, double subtree_Threshold);

	//remove similar or collapsed edges in an iteratively fashion
    void merge_collapsed_edges();

	//check if the child vertices of the current vertex can be merged
    bool check_overlap_child_vertex(Graph* i_Graph, SGraphVertexDescriptor i_dVertex);

	//check if the child vertices of the current vertex can be merged
    bool check_single_child_vertex(Graph* i_Graph, SGraphVertexDescriptor i_dVertex);

	//merge from source vertex to target vertex
    bool merge_vertices(Graph* i_Graph, SGraphVertexDescriptor i_dSource, SGraphVertexDescriptor i_dTarget, double i_wSource, double i_wTarget);



	/*-------------------------------------------------------------*/
	/*-----------method for graph attributes computation-----------*/
	/*-------------------------------------------------------------*/
	//compute the weights of edges in Delaunay graph
    void compute_delaunay_weight();

	//find and assign the root vertex in the input graph
    void compute_root_vertex(Graph* i_Graph);

	//compute the length of subtree for each vertex using a recursive method
    void compute_length_of_subtree(Graph* i_Graph, SGraphVertexDescriptor i_dVertex);

	//compute weights of edges according to the subtreelength of the end vertices
    void compute_graph_edges_weight(Graph* i_Graph);

	//compute the radius of rest edges according to the edges weight
    void compute_all_edges_radius(double trunkRadius);

	//compute the merge value between the source vertex and the target vertex
    double compute_merge_value(Graph* i_Graph, SGraphVertexDescriptor i_dSource, SGraphVertexDescriptor i_dTarget);



	/*-------------------------------------------------------------*/
	/*--------------method for point cloud processing -------------*/
	/*-------------------------------------------------------------*/
	//identify and centralize main branch points according to the density
    std::vector<Vector3D> centralize_main_points(easy3d::PointCloud* cloud);

	//compute to get the initial guess for trunk radius from the raw points
    void obtain_initial_radius(easy3d::PointCloud* cloud);



	/*-------------------------------------------------------------*/
	/*------------------method for branch fitting -----------------*/
	/*-------------------------------------------------------------*/
	//assign points to branch edges
    void assign_points_to_edges();

	//fit accurate cylinder to the trunk
    void fit_trunk();



	/*-------------------------------------------------------------*/
	/*------------------method for adding leaves ------------------*/
	/*-------------------------------------------------------------*/
	//find leaf vertex in the skeleton graph	
    std::vector<SGraphVertexDescriptor> find_end_vertices();

	//generate random leaves for each leaf vertex
    void generate_leaves(SGraphVertexDescriptor i_LeafVertex, double leafsize_Factor);



	/*-------------------------------------------------------------*/
	/*------------method for getting branch path ------------------*/
	/*-------------------------------------------------------------*/
	//find path for smoothing the graph
	void get_graph_for_smooth(std::vector<Path> &pathList);


    /*-------------------------------------------------------------*/
    /*------------ method for surface extraction ------------------*/
    /*-------------------------------------------------------------*/
    // mesh a generalized cylinder to a surface mesh
    void add_generalized_cylinder_to_model(
            easy3d::SurfaceMesh* mesh,
            const Branch& points,
            int slices);

private:
	/*store points and kd index*/
	Vector3D* Points_;
	KdTree* KDtree_;

	/*store initial and fine skeleton*/
    Graph   delaunay_;
    Graph   MST_;
    Graph   simplified_skeleton_;
    Graph   smoothed_skeleton_;

	/*store leaves*/
	std::vector<Leaf> VecLeaves_;

	/*store important vertex and geometrical attributes*/
	SGraphVertexDescriptor RootV_;
	Vector3D RootPos_;
	double TrunkRadius_;
	double TreeHeight_;
	double BoundingDistance_;

    bool   quiet_;
};

#endif
