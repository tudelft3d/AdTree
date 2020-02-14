/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <easy3d/algo/remove_duplication.h>

#include <cassert>

#include <easy3d/core/point_cloud.h>
#include <3rd_party/kd_tree/Vector3D.h>
#include <3rd_party/kd_tree/KdTree.h>


namespace easy3d {

    std::vector<PointCloud::Vertex> RemoveDuplication::apply(PointCloud *cloud, float epsilon) {
        const int maxBucketSize = 16;
        std::vector<vec3>& points = cloud->points();
        float* pointer = points[0];
        KdTree kd(reinterpret_cast<Vector3D*>(pointer), points.size(), maxBucketSize);

        std::vector<bool> keep(cloud->vertices_size(), true);

        double sqr_dist = epsilon * epsilon;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (keep[i]) {
                const vec3 &p = points[i];
                kd.queryRange(Vector3D(p.x, p.y, p.z), sqr_dist, true);
                int num = kd.getNOfFoundNeighbours();
                if (num > 1) {
                    for (int j = 1; j < num; ++j) {
                        int idx = kd.getNeighbourPositionIndex(j);
                        keep[idx] = 0;
                    }
                }
            }
        }

        std::vector<PointCloud::Vertex> points_to_remove;
        for (std::size_t i = 0; i < keep.size(); ++i) {
            if (!keep[i])
                points_to_remove.push_back(PointCloud::Vertex(i));
        }

        return points_to_remove;
    }

}
