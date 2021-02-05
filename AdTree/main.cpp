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
#include <easy3d/algo/remove_duplication.h>
#include <easy3d/util/file_system.h>

#include "skeleton.h"
#include "tree_viewer.h"

using namespace easy3d;

// returns the number of processed input files.
int batch_reconstruct(std::vector<std::string>& point_cloud_files, const std::string& output_folder) {
    int count(0);
    for (std::size_t i=0; i<point_cloud_files.size(); ++i) {
        const std::string& xyz_file = point_cloud_files[i];
        std::cout << "------------- " << i + 1 << "/" << point_cloud_files.size() << " -------------" << std::endl;
        std::cout << "processing xyz_file: " << xyz_file << std::endl;

        if (!file_system::is_directory(output_folder)) {
            if (file_system::create_directory(output_folder))
                std::cout << "created output directory '" << output_folder << "'" << std::endl;
            else {
                std::cerr << "failed creating output directory" << std::endl;
                return 0;
            }
        }

        // load point_cloud
        PointCloud *cloud = PointCloudIO::load(xyz_file);
        if (cloud) {
            std::cout << "cloud loaded. num points: " << cloud->n_vertices() << std::endl;

            // compute bbox
            Box3 box;
            auto points = cloud->get_vertex_property<vec3>("v:point");
            for (auto v : cloud->vertices())
                box.add_point(points[v]);

            // remove duplicated points
            const float threshold = box.diagonal() * 0.001f;
            const auto &points_to_remove = RemoveDuplication::apply(cloud, threshold);
            for (auto v : points_to_remove)
                cloud->delete_vertex(v);
            cloud->garbage_collection();
            std::cout << "removed too-close points. num points: " << cloud->n_vertices() << std::endl;
        }
        else {
            std::cerr << "failed to load point cloud from '" << xyz_file << "'" << std::endl;
            continue;
        }

        // reconstruct branches
        SurfaceMesh *mesh = new SurfaceMesh;
        const std::string &branch_filename = file_system::base_name(cloud->name()) + "_branches.obj";
        mesh->set_name(branch_filename);

        Skeleton *skeleton = new Skeleton();
        bool status = skeleton->reconstruct_branches(cloud, mesh);
        if (!status) {
            std::cerr << "failed in reconstructing branches" << std::endl;
            delete cloud;
            delete mesh;
            delete skeleton;
            continue;
        }

        // copy translation property from point_cloud to surface_mesh
        SurfaceMesh::ModelProperty<dvec3> prop = mesh->add_model_property<dvec3>("translation");
        prop[0] = cloud->get_model_property<dvec3>("translation")[0];

        // save branches model
        const std::string branch_file = output_folder + "/" + branch_filename;
        if (SurfaceMeshIO::save(branch_file, mesh)) {
            std::cout << "model of branches saved to: " << branch_file << std::endl;
            ++count;
        }
        else
            std::cerr << "failed in saving the model of branches" << std::endl;

        delete cloud;
        delete mesh;
        delete skeleton;
    }

    return count;
}


int main(int argc, char *argv[]) {
//    argc = 3;
//    argv[1] = "/Users/lnan/Projects/adtree/data";
//    argv[2] = "/Users/lnan/Projects/adtree/data-results";

    if (argc == 1){
        TreeViewer viewer;
        viewer.run();
        return EXIT_SUCCESS;
    }
    else if (argc == 3){
        std::string first_arg(argv[1]);
        std::string output_dir(argv[2]);
        if (file_system::is_file(output_dir)) {
            std::cerr << "second argument cannot be an existing file name (it must be a directory)" << std::endl;
            return EXIT_FAILURE;
        }

        if (file_system::is_file(first_arg)) {
            std::vector<std::string> cloud_files = {first_arg};
            return batch_reconstruct(cloud_files, output_dir) > 0;
        }
        else if (file_system::is_directory(first_arg)) {
            std::vector<std::string> entries;
            file_system::get_directory_entries(first_arg, entries, false);
            std::vector<std::string> cloud_files;
            for (const auto& file_name : entries) {
                if (file_name.size() > 3 && file_name.substr(file_name.size() - 3) == "xyz")
                    cloud_files.push_back(first_arg + "/" + file_name);
            }
            return batch_reconstruct(cloud_files, output_dir) > 0;
        }
    }
    else {
        std::cerr << "AdTree can be run in three modes, which can be selected based on arguments:" << std::endl;
        std::cerr << "  - GUI mode." << std::endl;
        std::cerr << "         Command: ./AdTree" << std::endl;
        std::cerr << "  - Single processing mode (i.e., processing a single point cloud file)." << std::endl;
        std::cerr << "         Command: ./AdTree <xyz_file_path> <output_directory>" << std::endl;
        std::cerr << "  - Batch processing mode (i.e., all *.xyz files in the input directory will be treated as input \n";
        std::cerr << "    for reconstruction and the reconstructed models will be save in the output directory.\n";
        std::cerr << "         Command: ./AdTree <xyz_files_directory> <output_directory>" << std::endl;
        return EXIT_FAILURE;
    }
}
