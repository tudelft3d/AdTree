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


#include "tree_viewer.h"
#include "skeleton.h"

#include <3rd_party/glew/include/GL/glew.h>		// Initialize with glewInit() 
#include <3rd_party/glfw/include/GLFW/glfw3.h>	// Include glfw3.h after our OpenGL definitions

#include <easy3d/viewer/drawable.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/camera.h>
#include <easy3d/viewer/soft_shadow.h>
#include <easy3d/util/dialogs.h>
#include <easy3d/util/file.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/viewer/shader_program.h>
#include <easy3d/viewer/shader_manager.h>
#include <easy3d/viewer/setting.h>

#include <iostream>
#include <fstream>


using namespace easy3d;

TreeViewer::TreeViewer()
    : ViewerImGui("AdTree")
    , skeleton_(nullptr)
{
    set_background_color(easy3d::vec3(1, 1, 1));

    camera_->setUpVector(vec3(0, 0, 1));
    camera_->setViewDirection(vec3(-1, 0, 0));
    camera_->showEntireScene();

    shadow_ = new SoftShadow(camera());
    shadow_->set_sample_pattern(SoftShadow::SamplePattern(2));
    shadow_->set_darkness(0.3f);
    shadow_->set_softness(0.9f);
    shadow_->set_background_color(background_color_);

    std::cout << usage() << std::endl;
}


TreeViewer::~TreeViewer()
{

}


void TreeViewer::cleanup() {
    if (skeleton_)
        delete skeleton_;

    if (shadow_)
        delete shadow_;

    ViewerImGui::cleanup();
}


easy3d::PointCloud* TreeViewer::cloud() const {
    if (models().empty())
        return nullptr;
    else
        return dynamic_cast<PointCloud*>(models()[0]);
}


easy3d::SurfaceMesh* TreeViewer::branches() const {
    if (models().size() < 2)
        return nullptr;
    else
        return dynamic_cast<SurfaceMesh*>(models()[1]);
}


easy3d::SurfaceMesh* TreeViewer::leaves() const {
    if (models().size() < 3)
        return nullptr;
    else
        return dynamic_cast<SurfaceMesh*>(models()[2]);
}


bool TreeViewer::key_press_event(int key, int modifiers) 
{
    if (key == GLFW_KEY_P && modifiers == GLFW_MOD_SHIFT) {
        if (!cloud())
            return false;
        cloud()->set_visible(!cloud()->is_visible());
		return true;
	}

    else if (key == GLFW_KEY_G && modifiers == GLFW_MOD_SHIFT) {
        if (!cloud())
            return false;

		//shift the visibility of the graph drawable
        easy3d::LinesDrawable* graph_drawable = cloud()->lines_drawable("graph");
        if (!graph_drawable)
            create_skeleton_drawable(ST_SMOOTHED);
        if (graph_drawable)
            graph_drawable->set_visible(!graph_drawable->is_visible());
		return true;
	}

	else if (key == GLFW_KEY_B && modifiers == GLFW_MOD_SHIFT)
	{
        if (!branches())
            return false;
        branches()->set_visible(!branches()->is_visible());
		return true;
	}

	else if (key == GLFW_KEY_L && modifiers == GLFW_MOD_SHIFT)
	{
        if (!leaves())
            return false;
        leaves()->set_visible(!leaves()->is_visible());
		return true;
	}

	else
		return Viewer::key_press_event(key, modifiers);
}


std::string TreeViewer::usage() const {
    return Viewer::usage() + std::string(
                "  Shift + P:       Show/Hide point cloud       \n"
                "  Shift + G:       Show/Hide skeleton          \n"
                "  Shift + B:       Show/Hide branches          \n"
                "  Shift + L:       Show/Hide leaves            \n"
    );
}


bool TreeViewer::open()
{
    for (auto m : models_)
        delete m;
    models_.clear();

    const std::vector<std::string> filetypes = {"*.xyz"};
    const std::string& file_name = FileDialog::open(filetypes, std::string(""));

    if (Viewer::open(file_name)) {
        set_title("AdTree - " + file::simple_name(cloud()->name()));
        fit_screen();
        return true;
    }
    return false;
}


bool TreeViewer::save() const {
    SurfaceMesh* mesh = branches();
    if (!mesh) {
        std::cerr << "branch model does not exist" << std::endl;
        return false;
    }

    const std::vector<std::string> filetypes = {"*.obj"};
    const std::string& file_name = FileDialog::save(filetypes, mesh->name());
    if (file_name.empty())
        return false;

    if (SurfaceMeshIO::save(file_name, mesh)) {
        std::cout << "file successfully saved" << std::endl;
        return true;
    }
    else
        return false;
}


void TreeViewer::export_skeleton() const {
    if (!branches() || !skeleton_) {
        std::cerr << "branch model does not exist" << std::endl;
        return;
    }

    const std::vector<std::string> filetypes = {"*.ske"};
    const std::string& initial_name = file::base_name(cloud()->name()) + "_skeleton.ske";
    const std::string& file_name = FileDialog::save(filetypes, initial_name);
    if (file_name.empty())
        return;

    const std::vector<Skeleton::Branch>& branches = skeleton_->get_branches_parameters();
    if (branches.empty())
        return;
    std::cout << "not implemented yet" << std::endl;
    //
    //
}


void TreeViewer::draw() {
    if (cloud()) {
        const mat4& MVP = camera_->modelViewProjectionMatrix();
        // camera position is defined in world coordinate system.
        const vec3& wCamPos = camera_->position();
        // it can also be computed as follows:
        //const vec3& wCamPos = invMV * vec4(0, 0, 0, 1);
        const mat4& MV = camera_->modelViewMatrix();
        const vec4& wLightPos = inverse(MV) * setting::light_position;

        if (cloud()->is_visible()) {
            ShaderProgram* program = program = ShaderManager::get_program("points_color");
            if (!program) {
                std::vector<ShaderProgram::Attribute> attributes;
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::POSITION, "vtx_position"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::COLOR, "vtx_color"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::NORMAL, "vtx_normal"));
                program = ShaderManager::create_program_from_files("points_color", attributes);
            }
            if (program) {
                program->bind();
                program->set_uniform("MVP", MVP);
                program->set_uniform("wLightPos", wLightPos);
                program->set_uniform("wCamPos", wCamPos);
                program->set_uniform("ssaoEnabled", false);
                for (auto m : models_) {
                    if (!m->is_visible())
                        continue;
                    for (auto d : m->points_drawables()) {
                        if (d->is_visible()) {
                            program->set_uniform("lighting", d->normal_buffer());
                            program->set_uniform("per_vertex_color", d->per_vertex_color() && d->color_buffer());
                            program->set_uniform("default_color", d->default_color());
                            d->draw(false);
                        }
                    }
                }
                program->release();
            }
        }

        easy3d::LinesDrawable* graph_drawable = cloud()->lines_drawable("graph");
        if (graph_drawable && graph_drawable->is_visible()) {
            ShaderProgram* program = ShaderManager::get_program("lines_color");
            if (!program) {
                std::vector<ShaderProgram::Attribute> attributes;
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::POSITION, "vtx_position"));
                attributes.push_back(ShaderProgram::Attribute(ShaderProgram::COLOR, "vtx_color"));
                program = ShaderManager::create_program_from_files("lines_color", attributes);
            }
            if (program) {
                program->bind();
                program->set_uniform("MVP", MVP);
                program->set_uniform("per_vertex_color", false);
                program->set_uniform("default_color", graph_drawable->default_color());
                graph_drawable->draw(false);
                program->release();
            }
        }
    }

    std::vector<TrianglesDrawable*> surfaces;
    if (branches() && branches()->is_visible()) {
        for (auto d : branches()->triangles_drawables())
            surfaces.push_back(d);
    }
    if (leaves() && leaves()->is_visible()) {
        for (auto d : leaves()->triangles_drawables())
            surfaces.push_back(d);
    }
    shadow_->draw(surfaces);
}


bool TreeViewer::create_skeleton_drawable(SkeletonType type)
{
    if (!cloud())
        return false;

	//get the skeleton graph to be rendered
    const Graph* skeleton = nullptr;
    switch (type) {
    case ST_DELAUNAY:
        skeleton = &(skeleton_->get_delaunay());
        break;
    case ST_MST:
        skeleton = &(skeleton_->get_mst());
        break;
    case ST_SIMPLIFIED:
        skeleton = &(skeleton_->get_simplified_skeleton());
        break;
    case ST_SMOOTHED:
        skeleton = &(skeleton_->get_smoothed_skeleton());
        break;
    }
    if (!skeleton)
	{
        std::cout << "skeleton does not exist" << std::endl;
		return false;
	}

	//create the vertices vector for rendering
	std::vector<easy3d::vec3> graph_points;
    std::pair<SGraphEdgeIterator, SGraphEdgeIterator> ep = edges(*skeleton);
	SGraphVertexDescriptor dVertex1, dVertex2;
	easy3d::vec3 pVertex1, pVertex2;
	for (SGraphEdgeIterator eIter = ep.first; eIter != ep.second; ++eIter)
	{
        dVertex1 = source(*eIter, *skeleton);
        dVertex2 = target(*eIter, *skeleton);
        pVertex1 = (*skeleton)[dVertex1].cVert;
        pVertex2 = (*skeleton)[dVertex2].cVert;
		graph_points.push_back(pVertex1);
		graph_points.push_back(pVertex2);
	}

	//initialize the line drawable object;
    easy3d::LinesDrawable* graph_drawable = cloud()->lines_drawable("graph");
	if (!graph_drawable)
        graph_drawable = cloud()->add_lines_drawable("graph");
	graph_drawable->update_vertex_buffer(graph_points);
	graph_drawable->set_per_vertex_color(false);
	graph_drawable->set_default_color(easy3d::vec3(0.0f, 0.0f, 0.0f));
	
	return true;
}


bool TreeViewer::reconstruct_skeleton() {
    if (!cloud()) {
        std::cout << "point cloud does not exist" << std::endl;
        return false;
    }

    if (skeleton_)
        delete skeleton_;
    skeleton_ = new Skeleton;

    SurfaceMesh* mesh = branches();
    if (mesh)
        mesh->clear();
    else {
        mesh = new SurfaceMesh;
        mesh->set_name(file::base_name(cloud()->name()) + "_branches.obj");
    }
    bool status = skeleton_->reconstruct_branches(cloud(), mesh);
    if (status) {
        if (!branches())
            add_model(mesh);

        cloud()->set_visible(false);
        return true;
    }

    return false;
}


bool TreeViewer::add_leaves() {
    if (!skeleton_) {
        std::cout << "please generate skeleton first!" << std::endl;
        return false;
    }

    SurfaceMesh* mesh = leaves();
    if (mesh)
        mesh->clear();
    else {
        mesh = new SurfaceMesh;
        mesh->set_name(file::base_name(cloud()->name()) + "_leaves.obj");
    }

    if (skeleton_->reconstruct_leaves(mesh)) {
        if (!leaves())
            add_model(mesh);

        easy3d::TrianglesDrawable* leaves_drawable = mesh->triangles_drawable("surface");
        if (leaves_drawable) {
            leaves_drawable->set_per_vertex_color(false);
            leaves_drawable->set_default_color(easy3d::vec3(0.50f, 0.83f, 0.20f));
        }

        return true;
    }

    return false;
}

