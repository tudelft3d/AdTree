/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*	https://3d.bk.tudelft.nl/liangliang/
*
*	This file is part of Easy3D. If it is useful in your research/work, 
*   I would be grateful if you show your appreciation by citing it:
*   ------------------------------------------------------------------
*           Liangliang Nan. 
*           Easy3D: a lightweight, easy-to-use, and efficient C++ 
*           library for processing and rendering 3D data. 2018.
*   ------------------------------------------------------------------
*
*	Easy3D is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	Easy3D is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include <easy3d/fileio/surface_mesh_io.h>

#include <clocale>

#include <easy3d/core/surface_mesh.h>
#include <easy3d/util/file_system.h>
#include <easy3d/util/stop_watch.h>


namespace easy3d {


	SurfaceMesh* SurfaceMeshIO::load(const std::string& file_name)
	{
		std::setlocale(LC_NUMERIC, "C");

        const std::string& ext = file_system::extension(file_name, true);

		SurfaceMesh* mesh = new SurfaceMesh;
		mesh->set_name(file_name);

		StopWatch w;

		bool success = false;
        if (ext == "obj")
            success = io::load_obj(file_name, mesh);
		else {
            std::cerr << "only obj format is supported" << std::endl;
            success = false;
		}

		if (!success || mesh->n_faces() == 0) {
			delete mesh;
			return nullptr;
		}

		std::cout << "load model done. time: " << w.time_string() << std::endl;

		return mesh;
	}


	bool SurfaceMeshIO::save(const std::string& file_name, const SurfaceMesh* mesh)
	{
		if (!mesh) {
			std::cerr << "surface mesh is null" << std::endl;
			return false;
		}

        std::string ext = file_system::extension(file_name, true);

		StopWatch w;

		bool success = false;
        if (ext == "obj")
            success = io::save_obj(file_name, mesh);
        else {
            std::cerr << "only obj format is supported" << std::endl;
            success = false;
		}

		std::cout << "save model done. time: " << w.time_string() << std::endl;

		return success;
	}


} // namespace easy3d
