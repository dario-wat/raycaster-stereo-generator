#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <boost/python.hpp>

namespace ster {

class Vector {
public:
    double x, y, z;
    static const Vector ZERO;

    Vector() : x(0), y(0), z(0) {}
    // Vector(double x, double y, double z) : x(x), y(y), z(z) {}
    int doit(double z) {return (int) z; }
    // Vector operator+(const Vector &other) const { return Vector(x+other.x, y+other.y, z+other.z); }
    // Vector operator-(const Vector &other) const { return Vector(x-other.x, y-other.y, z-other.z); }
    // Vector cross(const Vector &other) const {
    //     return Vector(y*other.z - z*other.y, z*other.x - x*other.z, x*other.y - y*other.x);
    // }
    // double operator*(const Vector &other) const { return x*other.x + y*other.y + z*other.z; }
    // Vector operator*(double m) const { return Vector(x*m, y*m, z*m); }
    // bool operator==(const Vector &other) const { return x == other.x && y == other.y && z == other.z; }
    // bool operator!=(const Vector &other) const { return !(*this == other); }
};

struct Ray {
    Vector a, b;
};

struct Triangle {
    Vector a, b, c;
};

int intersect_ray_triangle(const Ray &r, const Triangle &t, Vector &p);

}


using namespace boost::python;

BOOST_PYTHON_MODULE(teststuff) {
    class_<ster::Vector>("Vector", init<>())
        .def("doit", &ster::Vector::doit);
}

#endif