#include "geometry.h"

#include <sstream>
#include <iomanip>
#include <cmath>
#include <boost/python/tuple.hpp>
#include <boost/python/object.hpp>
#include <boost/python/list.hpp>

#include <iostream>

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

// Converts vector into a string
std::string ster::Vector::to_string() const {
    std::stringstream ss;
    ss << '[' << std::setprecision(3) << v[0] << ' ' << v[1] << ' ' << v[2] << ']';
    return ss.str();
}

// Rotates vectors and points around the axis of rotation.
// rotAxis     - the axis of rotation (it goes through the origin and is normalized)
// rotAngle    - angle of rotation
// vectors     - a list of vectors to rotate or points that need to be translated around the
//               origin since the axis of rotation goes through origin and the rotation
//               would not work otherwise.
boost::python::list ster::rotate_3d(
        const ster::Vector &rot_axis, double rot_angle, const boost::python::list &vectors) {
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
        ster::Vector vec = boost::python::extract<ster::Vector>(vectors[i]);
        list.append(ster::Vector(va*vec, vb*vec, vc*vec));
    }
    return list;
}  