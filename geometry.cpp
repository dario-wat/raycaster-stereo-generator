#include "geometry.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <boost/python/tuple.hpp>
#include <boost/python/object.hpp>
#include <boost/python/list.hpp>

const ster::Vector ster::Vector::ZERO = ster::Vector(0.0, 0.0, 0.0);
static const boost::python::object NONE = boost::python::object();

// Helper function to make creating of tuples easier.
template<typename A, typename B>
inline boost::python::tuple tuple_(A a, B b) {
    return boost::python::make_tuple(a, b);
}

// Intersection of ray and triangle in 3D. Returns 1 in tuple if it's found and intersection.
// If there is no intersection it will return something else.
boost::python::tuple ster::intersect_ray_triangle(const ster::Ray &ray, const ster::Triangle &triangle) {
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
boost::python::list ster::Vector::get_item_slice(const boost::python::slice &slice) const {
    boost::python::list result;
    boost::python::slice::range<std::array<double, 3>::const_iterator> range;
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
boost::python::list ster::rotate_3d(
        const ster::Vector &rot_axis, double rot_angle, const boost::python::list &vectors) {
    assert(rot_axis != ster::Vector::ZERO);
    ster::Vector rot_ax = rot_axis.normalize();
    double s = sin(rot_angle / 2.), c = cos(rot_angle / 2.);
    double q0 = c, q1 = s*rot_ax[0], q2 = s*rot_ax[1], q3 = s*rot_ax[2];
    double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
    double q01 = q0*q1, q02 = q0*q2, q03 = q0*q3, q12 = q1*q2, q13 = q1*q3, q23 = q2*q3;

    ster::Vector va(q00 + q11 - q22 - q33, 2*(q12 - q03), 2*(q13 + q02));
    ster::Vector vb(2*(q12 + q03), q00 - q11 + q22 - q33, 2*(q23 - q01));
    ster::Vector vc(2*(q13 - q02), 2*(q23 + q01), q00 - q11 - q22 + q33);

    boost::python::list list;
    int n = boost::python::len(vectors);
    for (int i = 0; i < n; i++) {
        if (vectors[i] == NONE) {
            list.append(NONE);
            continue;
        }
        ster::Vector vec = boost::python::extract<ster::Vector>(vectors[i]);
        list.append(ster::Vector(va*vec, vb*vec, vc*vec));
    }
    return list;
}

// Creates a grid, returns a list of grid points as ster::Vector-s
boost::python::list ster::create_grid(
        int width, int height,
        const ster::Vector &vec_x, const ster::Vector &vec_y, const ster::Vector &offset) {
    boost::python::list list;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            list.append(i*vec_x + j*vec_y + offset);
        }
    }
    return list;
}

// Intersects a ray and a fake rectangle which is defined with 2 triangles
boost::python::tuple ster::intersect_ray_fakerect(const ster::Ray &r, const ster::FakeRect &fr) {
    boost::python::tuple inter1 = ster::intersect_ray_triangle(r, fr.t1);
    boost::python::tuple inter2 = ster::intersect_ray_triangle(r, fr.t2);
    int f1 = boost::python::extract<int>(inter1[0]);
    int f2 = boost::python::extract<int>(inter2[0]);
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
boost::python::tuple ster::intersect_ray_scene(const ster::Ray &r, const boost::python::list &triangles) {
    int n = boost::python::len(triangles);
    double min_dist = std::numeric_limits<double>::max();
    boost::python::tuple curr_inters = boost::python::make_tuple(0, NONE, -1);
    for (int i = 0; i < n; i++) {
        ster::Triangle t = boost::python::extract<ster::Triangle>(triangles[i]);
        boost::python::tuple inters = ster::intersect_ray_triangle(r, t);
        int f = boost::python::extract<int>(inters[0]);
        if (f != 1) {
            continue;
        }
        ster::Vector inter_point = boost::python::extract<ster::Vector>(inters[1]);
        double dist = (r.a - inter_point).norm();       // this is actually squared distance
        if (dist > min_dist) {
            continue;
        }
        min_dist = dist;
        curr_inters = boost::python::make_tuple(f, inters[1], i);
    }
    return curr_inters;
}

// Simpler version of intersect_ray_scene which just checks if the ray intersects any
// triangle in the scene. This is also faster since it can finish earlier (if an intersection
// is found) and also does not need to perform distance measuring.
// Note: if debugging is required change the return value to the index of the intersected triangle.
// This is generally a bad idea, but I feel like it should not be needed
bool ster::intersects_scene(const ster::Ray &r, const boost::python::list &triangles, int ignore_idx) {
    int n = boost::python::len(triangles);
    for (int i = 0; i < n; i++) {
        if (i == ignore_idx) {
            continue;
        }
        ster::Triangle t = boost::python::extract<ster::Triangle>(triangles[i]);
        boost::python::tuple inters = ster::intersect_ray_triangle(r, t);
        int f = boost::python::extract<int>(inters[0]);
        if (f == 1) {
            return true;
        }
    }
    return false;
}

// Shoots rays from the source through the grid and intersects them with the scene. Each
// ray that is successfully intersected with the scene is then backprojected towards the drain
// and intersected with the drain rectangle. If there is no intersection or the ray is occluded
// on its way by other triangles in the scene, None is added as a coordinate. Otherwise
// the intersected coordinate of the back ray and drain triangle is added to the list.
// These points can then be further modified to reflect image coordinates.
boost::python::list ster::raycast(
        const boost::python::list &grid,
        const ster::Vector &source,
        const boost::python::list &scene_triangles,
        const ster::Vector &drain,
        const ster::FakeRect &drain_rect) {
    boost::python::list coordinates;
    int n = boost::python::len(grid);
    for (int i = 0; i < n; i++) {
        // Find intersection of ray from source and scene
        ster::Vector grid_point = boost::python::extract<ster::Vector>(grid[i]);
        ster::Ray source_ray = ster::Ray(source, grid_point);
        boost::python::tuple inters = ster::intersect_ray_scene(source_ray, scene_triangles);
        int found_inters = boost::python::extract<int>(inters[0]);
        if (found_inters != 1) {
            coordinates.append(NONE);
            continue;
        }

        // Find intersection in the drain grid from scene to drain
        ster::Vector inters_point = boost::python::extract<ster::Vector>(inters[1]);
        ster::Ray drain_ray = ster::Ray(inters_point, drain);
        boost::python::tuple drain_rect_inters = ster::intersect_ray_fakerect(drain_ray, drain_rect);
        int found_drain_inters = boost::python::extract<int>(drain_rect_inters[0]);
        if (found_drain_inters != 1) {
            coordinates.append(NONE);
            continue;
        }

        // Check if backprojected ray is occluded by something in the scene
        int triangle_idx = boost::python::extract<int>(inters[2]);
        if (ster::intersects_scene(drain_ray, scene_triangles, triangle_idx)) {
            coordinates.append(NONE);
            continue;
        }

        ster::Vector drain_rect_inters_point = boost::python::extract<ster::Vector>(drain_rect_inters[1]);
        coordinates.append(drain_rect_inters_point);
    }
    return coordinates;
}

// Moves the rectangle to the origin. Rotates the rectangle so that it is parallel
// with x-y plane. And then splits each points into 2 vectors.
boost::python::list ster::convert_coordinates_2d(
        const boost::python::list &coordinates,
        const ster::Vector &drain_rect_origin,
        const ster::Vector &scaled_basis_x,
        const ster::Vector &scaled_basis_y,
        double rot_angle) {
    ster::Vector plane_normal = scaled_basis_x.cross(scaled_basis_y);
    ster::Vector desired_normal = ster::Vector(0, 0, 1);
    ster::Vector rot_axis = desired_normal.cross(plane_normal);
    double correction_angle = atan2(rot_axis.norm(), plane_normal*desired_normal);

    boost::python::list basis_vectors;
    basis_vectors.append(scaled_basis_x);
    basis_vectors.append(scaled_basis_y);
    boost::python::list corrected_bases = ster::rotate_3d(rot_axis, -correction_angle, basis_vectors);
    corrected_bases = ster::rotate_3d(desired_normal, rot_angle, corrected_bases);

    int n = boost::python::len(coordinates);
    boost::python::list rotated_points;
    for (int i = 0; i < n; i++) {
        if (coordinates[i] == NONE) {
            rotated_points.append(NONE);
            continue;
        }
        ster::Vector point = boost::python::extract<ster::Vector>(coordinates[i]);
        rotated_points.append(point - drain_rect_origin);
    }
    rotated_points = ster::rotate_3d(rot_axis, -correction_angle, rotated_points);
    rotated_points = ster::rotate_3d(desired_normal, rot_angle, rotated_points);
    
    ster::Vector vec_x = boost::python::extract<ster::Vector>(corrected_bases[0]);
    ster::Vector vec_y = boost::python::extract<ster::Vector>(corrected_bases[1]);
    double xlen = vec_x[0];
    double ylen = vec_y[1];
    for (int i = 0; i < n; i++) {
        if (rotated_points[i] == NONE) {
            continue;
        }
        ster::Vector point = boost::python::extract<ster::Vector>(rotated_points[i]);
        rotated_points[i] = tuple_(point[0] / xlen, point[1] / ylen);
    }
    return rotated_points;
}