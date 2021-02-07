#ifndef ADTREE_TREE_VIEWER_H
#define ADTREE_TREE_VIEWER_H

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


#include "viewer_imgui.h"

#include <vector>


class Skeleton;

namespace easy3d {
    class PointCloud;
    class SurfaceMesh;
    class SoftShadow;
}


// The original easy3d viewer enbales multiple models. In TreeViewer, we allow only three:
//    - model #1: the point cloud
//    - model #2: the 3D model of tree branches
//    - model #3: the 3D model of leaves

class TreeViewer : public easy3d::ViewerImGui
{
public:
	TreeViewer();
    ~TreeViewer() override;

protected:

    virtual std::string usage() const override;

    void cleanup() override;

	//overload the key press event to conduct the modelling process
    bool key_press_event(int key, int modifiers) override;

    bool open() override;
    bool save() const override;

    void export_skeleton() const override;
    void export_leaves() const override;

    bool reconstruct_skeleton() override;
    bool add_leaves() override;

	//create the line drawable for skeleton graphs
	enum SkeletonType {
		ST_DELAUNAY,
		ST_MST,
        ST_SIMPLIFIED,
        ST_SMOOTHED
	};
    bool create_skeleton_drawable(SkeletonType type);

    easy3d::PointCloud*  cloud() const;
    easy3d::SurfaceMesh* branches() const;
    easy3d::SurfaceMesh* leaves() const;

    void draw() override;

private:
    easy3d::SoftShadow* shadow_;
    Skeleton*           skeleton_;
};

#endif
