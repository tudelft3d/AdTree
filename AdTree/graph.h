#ifndef ADTREE_GRAPH_H
#define ADTREE_GRAPH_H

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

#include <easy3d/types.h>


struct SGraphVertexProp
{
	easy3d::vec3  cVert;
	unsigned int nParent;
	double lengthOfSubtree;
};

struct SGraphEdgeProp
{
	double nWeight;
	double nRadius;
};


class Graph : public boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SGraphVertexProp, SGraphEdgeProp >
{
public:
	Graph() {}
};

//typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SGraphVertexProp, SGraphEdgeProp > SGraph;
typedef boost::graph_traits<Graph>::vertex_descriptor SGraphVertexDescriptor;
typedef boost::graph_traits<Graph>::edge_descriptor SGraphEdgeDescriptor;
typedef boost::graph_traits<Graph>::vertex_iterator SGraphVertexIterator;
typedef boost::graph_traits<Graph>::edge_iterator SGraphEdgeIterator;
typedef boost::graph_traits<Graph>::adjacency_iterator SGraphAdjacencyIterator;
typedef boost::graph_traits<Graph>::out_edge_iterator  SGraphOutEdgeIterator;


#endif


