#include "geometry.h"

#include <sstream>
#include <iomanip>
#include <cmath>
#include <boost/python/tuple.hpp>
#include <boost/python/object.hpp>

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
    ss << '[' << std::setprecision(3) << x << ' ' << y << ' ' << z << ']';
    return ss.str();
}