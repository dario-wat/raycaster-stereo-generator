#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <string>
#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

namespace ster {

class Vector {
public:
    double x, y, z;
    static const Vector ZERO;

    Vector(double x, double y, double z) : x(x), y(y), z(z) {}
    Vector operator+(const Vector &other) const { return Vector(x+other.x, y+other.y, z+other.z); }
    Vector operator-(const Vector &other) const { return Vector(x-other.x, y-other.y, z-other.z); }
    Vector cross(const Vector &other) const {
        return Vector(y*other.z - z*other.y, z*other.x - x*other.z, x*other.y - y*other.x);
    }
    double operator*(const Vector &other) const { return x*other.x + y*other.y + z*other.z; }
    Vector operator*(double m) const { return Vector(x*m, y*m, z*m); }
    bool operator==(const Vector &other) const { return x == other.x && y == other.y && z == other.z; }
    bool operator!=(const Vector &other) const { return !(*this == other); }

    std::string to_string() const;
};

struct Ray {
    Vector a, b;
    Ray(const Vector &a, const Vector &b) : a(a), b(b) {}
};

struct Triangle {
    Vector a, b, c;
    Triangle(const Vector &a, const Vector &b, const Vector &c) : a(a), b(b), c(c) {}
};

boost::python::tuple intersect_ray_triangle(const Ray &r, const Triangle &t);

}

// boost export of the stuff
using namespace boost::python;

BOOST_PYTHON_MODULE(geometry_cpp) {
    class_<ster::Vector>("Vector", init<double, double, double>())
        .def("__repr__", &ster::Vector::to_string)
        .def_readonly("x", &ster::Vector::x)
        .def_readonly("y", &ster::Vector::y)
        .def_readonly("z", &ster::Vector::z);
    class_<ster::Ray>("Ray", init<ster::Vector, ster::Vector>());
    class_<ster::Triangle>("Triangle", init<ster::Vector, ster::Vector, ster::Vector>());
    def("intersect_ray_triangle", ster::intersect_ray_triangle);
}

#endif