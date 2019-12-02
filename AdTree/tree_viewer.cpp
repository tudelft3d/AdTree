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
            create_skeleton_drawable(ST_SIMPLIFIED);
        if (graph_drawable) {
            bool currentVisible = graph_drawable->is_visible();
            graph_drawable->set_visible(!currentVisible);
        }
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
        "  Shift + P:       Show/Hide point cloud							\n"
        "  Shift + B:       Show/Hide branches                              \n"
        "  Shift + L:       Show/Hide leaves                                \n"
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

    if (branches() && branches()->is_visible()) {
        std::vector<TrianglesDrawable*> surfaces;
        for (auto m : models_) {
            if (m->is_visible()) {
            for (auto d : m->triangles_drawables())
                surfaces.push_back(d);
            }
        }
        shadow_->draw(surfaces);
    }
}


bool TreeViewer::create_skeleton_drawable(SkeletonType type)
{
    if (!cloud())
        return false;

	//get the skeleton graph to be rendered
	Graph* RenderGraph = nullptr;
	if (type == ST_DELAUNAY)
        RenderGraph = skeleton_->get_delaunay();
	else if (type == ST_MST)
        RenderGraph = skeleton_->get_mst();
	else if (type == ST_SIMPLIFIED)
        RenderGraph = skeleton_->get_fine_skeleton();
	if (!RenderGraph)
	{
		std::cout << "fails to render the graph!" << std::endl;
		return false;
	}

	//create the vertices vector for rendering
	std::vector<easy3d::vec3> graph_points;
	std::pair<SGraphEdgeIterator, SGraphEdgeIterator> ep = edges(*RenderGraph);
	SGraphVertexDescriptor dVertex1, dVertex2;
	easy3d::vec3 pVertex1, pVertex2;
	for (SGraphEdgeIterator eIter = ep.first; eIter != ep.second; ++eIter)
	{
		dVertex1 = source(*eIter, *RenderGraph);
		dVertex2 = target(*eIter, *RenderGraph);
		pVertex1 = (*RenderGraph)[dVertex1].cVert;
		pVertex2 = (*RenderGraph)[dVertex2].cVert;
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


bool TreeViewer::extract_branch_surface()
{
    if (!cloud())
        return false;

	//get the branch skeleton to be rendered
    Graph* branchGraph = skeleton_->get_fine_skeleton();
	if (!branchGraph)
	{
        std::cout << "skeleton doesn not exist!" << std::endl;
		return false;
	}

    SurfaceMesh* mesh = branches();
    if (mesh)
        mesh->clear();
    else {
        mesh = new SurfaceMesh;
        mesh->set_name(file::base_name(cloud()->name()) + "_branches.obj");
    }

	std::pair<SGraphEdgeIterator, SGraphEdgeIterator> ep = edges(*branchGraph);
    const int renderSlice = 10;
	for (SGraphEdgeIterator eIter = ep.first; eIter != ep.second; ++eIter)
	{
		SGraphVertexDescriptor dVertex1 = source(*eIter, *branchGraph);
		SGraphVertexDescriptor dVertex2 = target(*eIter, *branchGraph);
        easy3d::vec3 s = (*branchGraph)[dVertex1].cVert;
        easy3d::vec3 t = (*branchGraph)[dVertex2].cVert;
		double r = (*branchGraph)[*eIter].nRadius;
        add_cylinder_to_model(mesh, r, renderSlice, s, t);
    }

    if (!branches())
        add_model(mesh);

	return true;
}


void TreeViewer::add_cylinder_to_model(easy3d::SurfaceMesh* mesh, double radius, int slices, const easy3d::vec3 &s, const easy3d::vec3 &t)
{
	//find a vector perpendicular to the direction
	const easy3d::vec3 offset = t - s;
	const easy3d::vec3 axis = easy3d::normalize(offset);
	easy3d::vec3 perp = orthogonal(axis);
	perp.normalize();
	const easy3d::vec3 p = s + perp * radius;
	const double angle_interval = 2.0 * M_PI / slices;

    std::vector<easy3d::SurfaceMesh::Vertex> s_vertices, t_vertices;
	//find the points for all slices
	for (int i = 0; i < slices; ++i) {
        double angle = i * angle_interval;
        const easy3d::vec3 a = s + easy3d::mat4::rotation(axis, static_cast<float>(angle)) * (p - s);
        const easy3d::vec3 b = a + offset;
        SurfaceMesh::Vertex va = mesh->add_vertex(a);
        SurfaceMesh::Vertex vb = mesh->add_vertex(b);
        s_vertices.push_back(va);
        t_vertices.push_back(vb);
	}

    for (std::size_t i=0; i<s_vertices.size(); ++i) {
        mesh->add_triangle(s_vertices[i], s_vertices[(i+1) % s_vertices.size()], t_vertices[i]);
        mesh->add_triangle(t_vertices[i], s_vertices[(i+1) % s_vertices.size()], t_vertices[(i+1) % s_vertices.size()]);
    }
}


bool TreeViewer::reconstruct_skeleton() {
    if (!cloud()) {
        std::cout << "point cloud does not exist" << std::endl;
        return false;
    }

    if (skeleton_)
        delete skeleton_;
    skeleton_ = new Skeleton;

    if (!skeleton_->build_delaunay(cloud())) {
        std::cerr << "failed Delaunay Triangulation" << std::endl;
        return false;
    }

    //extract the minimum spanning tree
    if (!skeleton_->extract_mst()) {
        std::cerr << "failed extracting MST" << std::endl;
        return false;
    }

    //simplify the tree skeleton
    if (!skeleton_->refine_skeleton()) {
        std::cerr << "failed skeleton simplification" << std::endl;
        return false;
    }

    //generate branches
    if (!skeleton_->inflate_branches() || !smooth_branches()) {
        std::cerr << "failed fitting branches" << std::endl;
        return false;
    }

    return true;
}


bool TreeViewer::add_leaves() {
    if (!skeleton_ || !skeleton_->get_fine_skeleton()) {
        std::cout << "please generate skeleton first!" << std::endl;
        return false;
    }

    if (!skeleton_->add_leaves())
        return false;

    //get the leaf list to be rendered
    std::vector<Leaf> leaves_set = skeleton_->get_leaves();
    if (leaves_set.size() == 0)
        return false;

    SurfaceMesh* mesh = leaves();
    if (mesh)
        mesh->clear();
    else {
        mesh = new SurfaceMesh;
        mesh->set_name(file::base_name(cloud()->name()) + "_leaves.obj");
    }

    for (std::size_t i = 0; i < leaves_set.size(); i++) {
        const Leaf& iLeaf = leaves_set[i];
        //compute the center and major axis, minor axis of the leaf quad
        easy3d::vec3 pCenter((iLeaf.cPos + (0.5 * iLeaf.cDir * iLeaf.nRad)));
        easy3d::vec3 dirMajor(0.5 * iLeaf.cDir * iLeaf.nLength);
        easy3d::vec3 dirMinor(0.5 * easy3d::cross(iLeaf.cDir, iLeaf.cNormal)*iLeaf.nRad);
        //compute the corner coordinates
        const easy3d::vec3 a = pCenter - dirMajor - dirMinor;
        const easy3d::vec3 b = pCenter + dirMajor - dirMinor;
        const easy3d::vec3 c = pCenter + dirMajor + dirMinor;
        const easy3d::vec3 d = pCenter - dirMajor + dirMinor;
        SurfaceMesh::Vertex va = mesh->add_vertex(a);
        SurfaceMesh::Vertex vb = mesh->add_vertex(b);
        SurfaceMesh::Vertex vc = mesh->add_vertex(c);
        SurfaceMesh::Vertex vd = mesh->add_vertex(d);
        mesh->add_triangle(va, vb, vc);
        mesh->add_triangle(va, vc, vd);
    }

    if (!leaves())
        add_model(mesh);

    easy3d::TrianglesDrawable* leaves_drawable = leaves()->triangles_drawable("surface");
    if (leaves_drawable) {
        leaves_drawable->set_per_vertex_color(false);
        leaves_drawable->set_default_color(easy3d::vec3(0.50f, 0.83f, 0.20f));
    }

    return true;
}


// fit generalized cylinders: internaally a hermite curve with varying radius
bool TreeViewer::smooth_branches()
{
	//get the branch skeleton to be rendered
	Graph* branchGraph = skeleton_->get_fine_skeleton();
	if (!branchGraph)
	{
		std::cout << "skeleton doesn not exist!" << std::endl;
		return false;
	}
	SurfaceMesh* mesh = branches();
	if (mesh)
		mesh->clear();
	else {
		mesh = new SurfaceMesh;
		mesh->set_name(file::base_name(cloud()->name()) + "_branches.obj");
	}

	// get paths
	std::vector<Path> pathList;
	skeleton_->get_graph_for_smooth(pathList);
	
	// for each path get its coordinates and generate a smooth curve
    for (std::size_t n_path = 0; n_path < pathList.size(); ++n_path)
	{
		Path currentPath = pathList[n_path];
		std::vector<easy3d::vec3> interpolatedPoints;
        std::vector<double> interpolatedRadii;
        int numOfSlices = 20;
        std::vector<int> numOfSlicesCurrent;
		// retrieve the current path and its vertices
        for (std::size_t n_node = 0; n_node < currentPath.size() - 1; ++n_node)
		{
			SGraphVertexDescriptor sourceV = currentPath[n_node];
			SGraphVertexDescriptor targetV = currentPath[n_node + 1];
			easy3d::vec3 pSource = (*branchGraph)[sourceV].cVert;
			easy3d::vec3 pTarget = (*branchGraph)[targetV].cVert;
			float branchlength = easy3d::distance(pSource, pTarget);
            numOfSlicesCurrent.push_back(std::max(static_cast<int>(branchlength * numOfSlices), 2));

			// compute the tangents
			easy3d::vec3 tangentOfSorce;
			easy3d::vec3 tangentOfTarget;
			// if the source vertex is the root
			if (sourceV == (*branchGraph)[sourceV].nParent)
				tangentOfSorce = (pTarget - pSource).normalize();
			else
			{
				SGraphVertexDescriptor parentOfSource = (*branchGraph)[sourceV].nParent;
				tangentOfSorce = (pTarget - (*branchGraph)[parentOfSource].cVert).normalize();
			}
			// if the target vertex is leaf
			if ((out_degree(targetV, *branchGraph) == 1) && (targetV != (*branchGraph)[targetV].nParent))
				tangentOfTarget = (pTarget - pSource).normalize();
			else
			{
				SGraphVertexDescriptor childOfTarget = currentPath[n_node + 2];
				tangentOfTarget = ((*branchGraph)[childOfTarget].cVert - pSource).normalize();
			}
			tangentOfSorce *= 1 * branchlength;
			tangentOfTarget *= 1 * branchlength;

			//fit hermite curve
			easy3d::vec3 A = tangentOfTarget + tangentOfSorce + 2 * (pSource - pTarget);
			easy3d::vec3 B = 3 * (pTarget - pSource) - 2 * tangentOfSorce - tangentOfTarget;
			easy3d::vec3 C = tangentOfSorce;
			easy3d::vec3 D = pSource;
			SGraphEdgeDescriptor currentE = edge(sourceV, targetV, *branchGraph).first;
			double sourceRadius = (*branchGraph)[currentE].nRadius;
			double targetRadius = sourceRadius;
			SGraphVertexDescriptor ParentVert = (*branchGraph)[sourceV].nParent;
			if (ParentVert != sourceV)
			{
				SGraphEdgeDescriptor ParentEdge = edge(ParentVert, sourceV, *branchGraph).first;
				sourceRadius = (*branchGraph)[ParentEdge].nRadius;
			}
			double deltaOfRadius = (sourceRadius - targetRadius) / numOfSlicesCurrent[numOfSlicesCurrent.size() - 1];
			//generate interpolated points
            for (std::size_t n = 0; n < numOfSlicesCurrent[numOfSlicesCurrent.size() - 1]; ++n)
			{
                double t = static_cast<double>(static_cast<double>(n) / numOfSlicesCurrent[numOfSlicesCurrent.size() - 1]);
				easy3d::vec3 point = A * t*t*t + B * t*t + C * t + D;
				interpolatedPoints.push_back(point);
				interpolatedRadii.push_back(sourceRadius - n * deltaOfRadius);
			}
		}
		//push back the last vertex
		SGraphVertexDescriptor endV = currentPath.back();
		interpolatedPoints.push_back((*branchGraph)[endV].cVert);
		interpolatedRadii.push_back(0);
		
		//start rendering for the current branch graph path
        const int slices = 10;
        add_generalized_cylinder_to_model(mesh, interpolatedRadii, interpolatedPoints, slices);

    }

	if (!branches())
		add_model(mesh);

	return true;
}



void TreeViewer::add_generalized_cylinder_to_model(easy3d::SurfaceMesh *mesh, const std::vector<double> &radius, const std::vector<easy3d::vec3> &points, int slices)
{
    if (points.size() < 2) {
        std::cerr << "two few points to represent a generalized cylinder" << std::endl;
        return;
    }

    typedef  std::vector<easy3d::SurfaceMesh::Vertex> CrossSection;
    std::vector< CrossSection > crosssections;
    easy3d::vec3 perp;
    for (std::size_t np = 0; np < points.size() - 1; np++)
    {
        easy3d::vec3 s = points[np];
        easy3d::vec3 t = points[np + 1];
        double r = radius[np];

        //find a vector perpendicular to the direction
        const easy3d::vec3 offset = t - s;
        const easy3d::vec3 axis = easy3d::normalize(offset);
        if (np == 0) {
            easy3d::vec3 tmp = orthogonal(axis);
            tmp.normalize();
            perp = tmp;
        }
        else {
            const easy3d::vec3 p = easy3d::Plane3(s, axis).projection(s + perp);
            perp = p - s;
            perp.normalize();
        }

        const easy3d::vec3 p = s + perp * r;
        const double angle_interval = 2.0 * M_PI / slices;
        //find the points for all slices
        CrossSection cs;
        for (int sli = 0; sli < slices; ++sli) {
            double angle = sli * angle_interval;
            const easy3d::vec3 a = s + easy3d::mat4::rotation(axis, static_cast<float>(angle)) * (p - s);
            easy3d::SurfaceMesh::Vertex v = mesh->add_vertex(a);
            cs.push_back(v);
        }
        crosssections.push_back(cs);
    }

    for (std::size_t nx = 0; nx < crosssections.size() - 1; ++nx) {
        const CrossSection& cs_curr = crosssections[nx];
        const CrossSection& cs_next = crosssections[nx+1];
        for (std::size_t ny = 0; ny < cs_curr.size(); ++ny) {
            mesh->add_triangle(cs_curr[ny], cs_curr[(ny+1) % cs_curr.size()], cs_next[ny]);
            mesh->add_triangle(cs_next[ny], cs_curr[(ny+1) % cs_curr.size()], cs_next[(ny+1) % cs_curr.size()]);
        }
    }
}
