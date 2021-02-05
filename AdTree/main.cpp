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


#include <iostream>

#include <easy3d/core/graph.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/types.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/util/file_system.h>

#include "skeleton.h"
#include "tree_viewer.h"

using namespace easy3d;

int batch_mode(std::string xyz_file, std::string export_folder){
    std::cout << "xyz_file : " << xyz_file << std::endl;
    std::cout << "export folder : " << export_folder << std::endl;

    if (!file_system::is_directory(export_folder)){
        file_system::create_directory(export_folder);
    }

    // load point_cloud
    PointCloud* cloud = PointCloudIO::load(xyz_file);
    if (!cloud) {
        std::cerr << "Cloud fails to load." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "cloud loaded. num vertices: " << cloud->n_vertices() << std::endl;

    // reconstruct branches
    SurfaceMesh* mesh = new SurfaceMesh;
    const std::string& branch_filename = file_system::base_name(cloud->name()) + "_branches.obj";
    mesh->set_name(branch_filename);

    Skeleton* skeleton_ = new Skeleton();
    bool status = skeleton_->reconstruct_branches(cloud, mesh);

    if (!status) {
        std::cerr << "branch model does not exist" << std::endl;
        return EXIT_FAILURE;
    }

    // copy translation property from point_cloud to surface_mesh
    SurfaceMesh::ModelProperty<dvec3> prop = mesh->add_model_property<dvec3>("translation");
    prop[0] = cloud->get_model_property<dvec3>("translation")[0];

    // save branches model
    auto branch_file = export_folder + "/" + branch_filename;
    std::cout << "branch file will be saved at : " << branch_file << std::endl;

    if (!SurfaceMeshIO::save(branch_file, mesh)){
        std::cerr << "save branch_file failed" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "save branch model done." << std::endl;
    return EXIT_SUCCESS;
}


int main(int argc, char *argv[]) {
    if (argc == 1){
        TreeViewer viewer;
        viewer.run();

        return EXIT_SUCCESS;
    }
    else if (argc == 3){
        std::string xyz_file(argv[1]);
        std::string export_folder(argv[2]);

        return batch_mode(xyz_file, export_folder);
    }
    else {
        std::cerr << "AdTree can be run in two modes, which can be selected based on arguments:" << std::endl;
        std::cerr << "- no arguments to open the GUI." << std::endl;
        std::cerr << "      Example:" << std::endl;
        std::cerr << "          `AdTree`" << std::endl;
        std::cerr << "- exactly two arguments to run AdTree in batch mode from the command line." << std::endl;
        std::cerr << "      In batch mode, AdTree expects: `./AdTree <xyz_file_path> <output_folder>`" << std::endl;
        std::cerr << "      The <output_folder> will be created if it doesn't exist." << std::endl;
        std::cerr << "      Example:" << std::endl;
        std::cerr << "          `AdTree \"./data/tree1.xyz\" \"./export_branches_models\"`" << std::endl;
        return EXIT_FAILURE;
    }
}
