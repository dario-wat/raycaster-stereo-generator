#include "geometry.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <tuple>
#include <unordered_set>
#include <cmath>
#include <boost/python/tuple.hpp>
#include <boost/python/object.hpp>
#include <boost/python/list.hpp>

namespace bp = boost::python;

const ster::Vector ster::Vector::ZERO = ster::Vector(0.0, 0.0, 0.0);
static const bp::object NONE = bp::object();
static const std::vector<std::tuple<int, int>> N8 = {
    std::make_tuple(-1, -1), std::make_tuple(1, -1), std::make_tuple(0, -1),
    std::make_tuple(-1,  0), std::make_tuple(1,  0), std::make_tuple(0,  1),
    std::make_tuple(-1,  1), std::make_tuple(1,  1)};

// Helper function to make creating of tuples easier.
template<typename A, typename B>
inline bp::tuple tuple_(A a, B b) {
    return bp::make_tuple(a, b);
}

// Creates list of size n initialized with given value
inline bp::list init_python_list(int n, bp::object value=NONE) {
    bp::list li;
    li.append(value);
    li *= n;
    return li;
}

// Makes a copy of a boost::python::list
bp::list copy_python_list(const bp::list &li) {
    int n = bp::len(li);
    bp::list li_copy = init_python_list(n);
    for (int i = 0; i < n; i++) {
        li_copy[i] = li[i];
    }
    return li_copy;
}

// boost::python::list to std::vector
template<typename T>
std::vector<T> to_vector(const bp::list &li) {
    int n = bp::len(li);
    std::vector<T> vec(n);
    for (int i = 0; i < n; i++) {
        vec[i] = bp::extract<T>(li[i]);
    }
    return vec;
}

// std::vector to boost::python::list
template<typename T>
bp::list to_python_list(const std::vector<T> &vec) {
    bp::list li;
    for (auto it = vec.begin(); it != vec.end(); it++) {
        li.append(*it);
    }
    return li;
}

// Intersection of ray and triangle in 3D. Returns 1 in tuple if it's found and intersection.
// If there is no intersection it will return something else.
bp::tuple ster::intersect_ray_triangle(const ster::Ray &ray, const ster::Triangle &triangle) {
    // triangle vectors
    ster::Vector u = triangle.b - triangle.a;
    ster::Vector v = triangle.c - triangle.a;
    ster::Vector n = u.cross(v);
    if (n == ster::Vector::ZERO) {      // degenerate triangle
        return tuple_(-1, NONE);
    }

    // ray vectors
    ster::Vector d = ray.b - ray.a;         // direction
    ster::Vector w0 = ray.a - triangle.a;
    double a = -(n * w0);
    double b = n * d;
    if (fabs(b) < 1e-6) {               // ray is parallel to the triangle
        if (a == 0) {
            return tuple_(2, NONE);
        }
        return tuple_(0, NONE);
    }

    double r = a / b;
    if (r < 0.0) {                      // no intersection, ray goes away from the triangle
        return tuple_(0, NONE);
    }

    // intersection of a ray and a plane
    ster::Vector intersection = ray.a + d*r;

    // checking if intersection is inside triangle
    double uu = u * u;
    double uv = u * v;
    double vv = v * v;
    ster::Vector w = intersection - triangle.a;
    double wu = w * u;
    double wv = w * v;
    double D = uv*uv - uu*vv;

    // test parametric coordinates
    double s = (uv*wv - vv*wu) / D;
    if (s < 0.0 || s > 1.0) {       // intersection is outside of the triangle
        return tuple_(0, NONE);
    }

    double t = (uv*wu - uu*wv) / D;
    if (t < 0.0 || s + t > 1.0) {   // intersection is outside of the triangle
        return tuple_(0, NONE);
    }

    return tuple_(1, intersection);
}

// Used for __getitem__ slice
bp::list ster::Vector::get_item_slice(const bp::slice &slice) const {
    bp::list result;
    bp::slice::range<std::array<double, 3>::const_iterator> range;
    try {
        range = slice.get_indices(v.begin(), v.end());
    } catch (std::invalid_argument) {
        return result;
    }
    for (; range.start != range.stop; std::advance(range.start, range.step)) {
        result.append(*range.start);
    }
    result.append(*range.start); // Handle last item.
    return result;
}

// Converts vector into a string
std::string ster::Vector::to_string() const {
    std::stringstream ss;
    ss << '[' << std::setprecision(3) << v[0] << ' ' << v[1] << ' ' << v[2] << ']';
    return ss.str();
}

// Rotates vectors and points around the axis of rotation. Skips None.
// rotAxis     - the axis of rotation (it goes through the origin and is normalized)
// rotAngle    - angle of rotation
// vectors     - a list of vectors to rotate or points that need to be translated around the
//               origin since the axis of rotation goes through origin and the rotation
//               would not work otherwise.
bp::list ster::rotate_3d(
        const ster::Vector &rot_axis, double rot_angle, const bp::list &vectors) {
    if (rot_axis == ster::Vector::ZERO) {
        // No rotation, strange way to do this
        return vectors;
    }
    // assert(rot_axis != ster::Vector::ZERO);
    ster::Vector rot_ax = rot_axis.normalize();
    double s = sin(rot_angle / 2.), c = cos(rot_angle / 2.);
    double q0 = c, q1 = s*rot_ax[0], q2 = s*rot_ax[1], q3 = s*rot_ax[2];
    double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
    double q01 = q0*q1, q02 = q0*q2, q03 = q0*q3, q12 = q1*q2, q13 = q1*q3, q23 = q2*q3;

    ster::Vector va(q00 + q11 - q22 - q33, 2*(q12 - q03), 2*(q13 + q02));
    ster::Vector vb(2*(q12 + q03), q00 - q11 + q22 - q33, 2*(q23 - q01));
    ster::Vector vc(2*(q13 - q02), 2*(q23 + q01), q00 - q11 - q22 + q33);

    bp::list list;
    int n = bp::len(vectors);
    for (int i = 0; i < n; i++) {
        if (vectors[i] == NONE) {
            list.append(NONE);
            continue;
        }
        ster::Vector vec = bp::extract<ster::Vector>(vectors[i]);
        list.append(ster::Vector(va*vec, vb*vec, vc*vec));
    }
    return list;
}

// Creates a grid, returns a list of grid points as ster::Vector-s
bp::list ster::create_grid(
        int width, int height,
        const ster::Vector &vec_x, const ster::Vector &vec_y, const ster::Vector &offset) {
    bp::list list;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            list.append(i*vec_x + j*vec_y + offset);
        }
    }
    return list;
}

// Intersects a ray and a fake rectangle which is defined with 2 triangles
bp::tuple ster::intersect_ray_fakerect(const ster::Ray &r, const ster::FakeRect &fr) {
    bp::tuple inter1 = ster::intersect_ray_triangle(r, fr.t1);
    bp::tuple inter2 = ster::intersect_ray_triangle(r, fr.t2);
    int f1 = bp::extract<int>(inter1[0]);
    int f2 = bp::extract<int>(inter2[0]);
    if (f1 == 1 && f2 == 1) {
        throw "Should not have both triangle intersections";
    }
    if (f1 == 1) {
        return inter1;
    }
    return inter2;
}

// Finds an intersection between a ray and a scene. If the scene is setup correctly there should
// always be at least 1 intersection. Since ther can be more, the function will return
// the closest intersection.
bp::tuple ster::intersect_ray_scene(const ster::Ray &r, const bp::list &triangles) {
    int n = bp::len(triangles);
    double min_dist = std::numeric_limits<double>::max();
    bp::tuple curr_inters = bp::make_tuple(0, NONE, -1);
    for (int i = 0; i < n; i++) {
        ster::Triangle t = bp::extract<ster::Triangle>(triangles[i]);
        bp::tuple inters = ster::intersect_ray_triangle(r, t);
        int f = bp::extract<int>(inters[0]);
        if (f != 1) {
            continue;
        }
        ster::Vector inter_point = bp::extract<ster::Vector>(inters[1]);
        double dist = (r.a - inter_point).norm();       // this is actually squared distance
        if (dist > min_dist) {
            continue;
        }
        min_dist = dist;
        curr_inters = bp::make_tuple(f, inters[1], i);
    }
    return curr_inters;
}

// Simpler version of intersect_ray_scene which just checks if the ray intersects any
// triangle in the scene. This is also faster since it can finish earlier (if an intersection
// is found) and also does not need to perform distance measuring.
// Note: if debugging is required change the return value to the index of the intersected triangle.
// This is generally a bad idea, but I feel like it should not be needed
bool ster::intersects_scene(const ster::Ray &r, const bp::list &triangles, int ignore_idx) {
    int n = bp::len(triangles);
    for (int i = 0; i < n; i++) {
        if (i == ignore_idx) {
            continue;
        }
        ster::Triangle t = bp::extract<ster::Triangle>(triangles[i]);
        bp::tuple inters = ster::intersect_ray_triangle(r, t);
        int f = bp::extract<int>(inters[0]);
        if (f == 1) {
            return true;
        }
    }
    return false;
}

template <typename T>
void fill_set(std::unordered_set<T> &set, const bp::list &li) {
    int n = bp::len(li);
    for (int i = 0; i < n; i++) {
        T item = bp::extract<T>(li[i]);
        set.insert(item);
    }
}

// Shoots rays from the source through the grid and intersects them with the scene. Each
// ray that is successfully intersected with the scene is then backprojected towards the drain
// and intersected with the drain rectangle. If there is no intersection or the ray is occluded
// on its way by other triangles in the scene, None is added as a coordinate. Otherwise
// the intersected coordinate of the back ray and drain triangle is added to the list.
// These points can then be further modified to reflect image coordinates.
bp::tuple ster::raycast(
        const bp::list &grid,
        const ster::Vector &source,
        const bp::list &scene_triangles,
        const ster::Vector &drain,
        const ster::FakeRect &drain_rect,
        const bp::list background_triangles) {
    std::unordered_set<int> bg_triangles;
    fill_set(bg_triangles, background_triangles);
    bp::list coordinates;
    bp::list depths_source;
    bp::list foreground;
    int n = bp::len(grid);
    for (int i = 0; i < n; i++) {
        // Find intersection of ray from source and scene
        ster::Vector grid_point = bp::extract<ster::Vector>(grid[i]);
        ster::Ray source_ray = ster::Ray(source, grid_point);
        bp::tuple inters = ster::intersect_ray_scene(source_ray, scene_triangles);
        int found_inters = bp::extract<int>(inters[0]);
        if (found_inters != 1) {
            coordinates.append(NONE);
            foreground.append(false);
            depths_source.append(0.0);
            continue;
        }

        int triangle_idx = bp::extract<int>(inters[2]);
        if (bg_triangles.find(triangle_idx) != bg_triangles.end()) {
            foreground.append(false);
        } else {
            foreground.append(true);
        }

        // Find intersection in the drain grid from scene to drain
        ster::Vector inters_point = bp::extract<ster::Vector>(inters[1]);
        depths_source.append(sqrt((inters_point - source)*(inters_point - source)));
        ster::Ray drain_ray = ster::Ray(inters_point, drain);
        bp::tuple drain_rect_inters = ster::intersect_ray_fakerect(drain_ray, drain_rect);
        int found_drain_inters = bp::extract<int>(drain_rect_inters[0]);
        if (found_drain_inters != 1) {
            coordinates.append(NONE);
            continue;
        }

        // Check if backprojected ray is occluded by something in the scene
        if (ster::intersects_scene(drain_ray, scene_triangles, triangle_idx)) {
            coordinates.append(NONE);
            continue;
        }

        ster::Vector drain_rect_inters_point = bp::extract<ster::Vector>(drain_rect_inters[1]);
        coordinates.append(drain_rect_inters_point);
    }
    return bp::make_tuple(coordinates, depths_source, foreground);
}

// Finds the depth from source and shooting through grid with points in the scene.
bp::list ster::depth_to_scene(
        const bp::list &grid,
        const ster::Vector &source,
        const bp::list &scene_triangles) {
    bp::list depths;
    int n = bp::len(grid);
    for (int i = 0; i < n; i++) {
        // Find intersection of ray from source and scene
        ster::Vector grid_point = bp::extract<ster::Vector>(grid[i]);
        ster::Ray source_ray = ster::Ray(source, grid_point);
        bp::tuple inters = ster::intersect_ray_scene(source_ray, scene_triangles);
        int found_inters = bp::extract<int>(inters[0]);
        if (found_inters != 1) {
            depths.append(0.0);
            continue;
        }

        ster::Vector inters_point = bp::extract<ster::Vector>(inters[1]);
        depths.append(sqrt((inters_point - source)*(inters_point - source)));
    }
    return depths;
}

// Moves the rectangle to the origin. Rotates the rectangle so that it is parallel
// with x-y plane. And then splits each points into 2 vectors.
bp::list ster::convert_coordinates_2d(
        const bp::list &coordinates,
        const ster::Vector &drain_rect_origin,
        const ster::Vector &scaled_basis_x,
        const ster::Vector &scaled_basis_y,
        double rot_angle) {
    ster::Vector plane_normal = scaled_basis_x.cross(scaled_basis_y);
    ster::Vector desired_normal = ster::Vector(0, 0, 1);
    ster::Vector rot_axis = desired_normal.cross(plane_normal);
    double correction_angle = atan2(rot_axis.norm(), plane_normal*desired_normal);

    bp::list basis_vectors;
    basis_vectors.append(scaled_basis_x);
    basis_vectors.append(scaled_basis_y);
    bp::list corrected_bases = ster::rotate_3d(rot_axis, -correction_angle, basis_vectors);
    corrected_bases = ster::rotate_3d(desired_normal, rot_angle, corrected_bases);

    int n = bp::len(coordinates);
    bp::list rotated_points;
    for (int i = 0; i < n; i++) {
        if (coordinates[i] == NONE) {
            rotated_points.append(NONE);
            continue;
        }
        ster::Vector point = bp::extract<ster::Vector>(coordinates[i]);
        rotated_points.append(point - drain_rect_origin);
    }
    rotated_points = ster::rotate_3d(rot_axis, -correction_angle, rotated_points);
    rotated_points = ster::rotate_3d(desired_normal, rot_angle, rotated_points);
    
    ster::Vector vec_x = bp::extract<ster::Vector>(corrected_bases[0]);
    ster::Vector vec_y = bp::extract<ster::Vector>(corrected_bases[1]);
    double xlen = vec_x[0];
    double ylen = vec_y[1];
    for (int i = 0; i < n; i++) {
        if (rotated_points[i] == NONE) {
            continue;
        }
        ster::Vector point = bp::extract<ster::Vector>(rotated_points[i]);
        rotated_points[i] = tuple_(point[0] / xlen, point[1] / ylen);
    }
    return rotated_points;
}

// To follow the logic of python functions and images, the first coordinate is y and
// the second is x. Returns coordinates of all pixels that are zero.
std::vector<std::tuple<int, int>> nonzero(const bp::list &image_mask, int w, int h) {
    std::vector<std::tuple<int, int>> nonzero_coordinates;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int pxl = bp::extract<int>(image_mask[y*w+x]);
            if (pxl != 0) {
                nonzero_coordinates.push_back(std::make_tuple(y, x));
            }
        }
    }
    return nonzero_coordinates;
}

// Fills in occluded pixels by the mean value of neighbors.
bp::list ster::missing_interpolation(
        const bp::list &image_orig, int w, int h, int n_iter, const bp::list &occl_map) {
    std::vector<int> image_to = to_vector<int>(image_orig);
    std::vector<int> image_from = to_vector<int>(image_orig);
    std::vector<int> occl_map_from = to_vector<int>(occl_map);
    std::vector<int> occl_map_to = to_vector<int>(occl_map);
    std::vector<std::tuple<int, int>> occl_from = nonzero(occl_map, w, h);

    for (int iter = 0; iter < n_iter; iter++) {
        std::vector<std::tuple<int, int>> occl_to;
        for (auto it_occl = occl_from.begin(); it_occl != occl_from.end(); it_occl++) {
            int y = std::get<0>(*it_occl), x = std::get<1>(*it_occl);
            double sum = 0.0;
            int count = 0;
            for (auto it = N8.begin(); it != N8.end(); it++) {
                int xn = x + std::get<0>(*it), yn = y + std::get<1>(*it);
                if (xn < 0 || yn < 0 || xn >= w || yn >= h || occl_map_from[yn*w+xn]) {
                    continue;
                }
                sum += image_from[yn*w+xn];
                count++;
            }
            if (count != 0) {
                image_to[y*w+x] = sum / count;
                occl_map_to[y*w+x] = 0;
            } else {
                occl_to.push_back(*it_occl);
            }
        }

        image_from = image_to;
        occl_map_from = occl_map_to;
        occl_from = occl_to;
    }
    return to_python_list(image_from);
}
