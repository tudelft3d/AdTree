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

#include <easy3d/algo/point_cloud_simplification.h>

#include <cassert>
#include <set>

#include <easy3d/core/point_cloud.h>


namespace easy3d {


    namespace internal {
        /// Utility class for grid simplification of point set.
        /// LessEpsilonPoints defines a 3D points order: two points are equal
        /// iff they belong to the same cell of a grid of cell size = epsilon.
        template<class Point>
        class LessEpsilonPoints {
        public:
            LessEpsilonPoints(float epsilon) : m_epsilon(epsilon) {
                assert(epsilon > 0);
            }

            // Round points to multiples of m_epsilon, then compare.
            bool operator()(const Point &a, const Point &b) const {
                const vec3 *a_n = (a.pos);
                const vec3 *b_n = (b.pos);

                vec3 rounded_a(round_epsilon(a_n->x, m_epsilon), round_epsilon(a_n->y, m_epsilon),
                               round_epsilon(a_n->z, m_epsilon));
                vec3 rounded_b(round_epsilon(b_n->x, m_epsilon), round_epsilon(b_n->y, m_epsilon),
                               round_epsilon(b_n->z, m_epsilon));

                //return (rounded_a < rounded_b);
                if (rounded_a.x < rounded_b.x)
                    return true;
                else if (rounded_a.x == rounded_b.x) {
                    if (rounded_a.y < rounded_b.y)
                        return true;
                    else if (rounded_a.y == rounded_b.y) {
                        if (rounded_a.z < rounded_b.z)
                            return true;
                    }
                }

                return false;
            }

        private:
            // Round number to multiples of epsilon
            static inline float round_epsilon(float value, float epsilon) {
                return std::floor(value / epsilon) * epsilon;
            }

        private:
            float m_epsilon;
        };
    }


    std::vector<PointCloud::Vertex> PointCloudSimplification::grid_simplification(PointCloud *cloud, float epsilon) {
        assert(epsilon > 0);

        struct Point {
            const vec3 *pos;
            PointCloud::Vertex vertex;
        };

        // Merges points which belong to the same cell of a grid of cell size = epsilon.
        // points_to_keep will contain 1 point per cell; the others will be in points_to_remove.
        std::set<Point, internal::LessEpsilonPoints<Point> > points_to_keep(epsilon);
        std::vector<PointCloud::Vertex> points_to_remove;

        const auto &points = cloud->points();
        for (auto v : cloud->vertices()) {
            Point p;
            p.pos = &(points[v.idx()]);
            p.vertex = v;
            const auto &result = points_to_keep.insert(p);
            if (!result.second) // if not inserted
                points_to_remove.push_back(v);
        }

        return points_to_remove;
    }


}
