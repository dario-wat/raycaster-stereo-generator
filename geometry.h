#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <string>
#include <array>
#include <cmath>
#include <boost/python.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/list.hpp>
#include <boost/python/slice.hpp>

namespace ster {

class Vector {
public:
    const std::array<double, 3> v;
    static const Vector ZERO;

    // Constructor
    Vector(double x, double y, double z) : v{x, y, z} {}
    
    // Operators for vector logic
    Vector operator+(const Vector &o) const { return Vector(v[0]+o.v[0], v[1]+o.v[1], v[2]+o.v[2]); }
    Vector operator-(const Vector &o) const { return Vector(v[0]-o.v[0], v[1]-o.v[1], v[2]-o.v[2]); }
    double operator*(const Vector &o) const { return v[0]*o.v[0] + v[1]*o.v[1] + v[2]*o.v[2]; }
    Vector operator*(double m) const { return Vector(v[0]*m, v[1]*m, v[2]*m); }
    Vector operator/(double m) const { return Vector(v[0]/m, v[1]/m, v[2]/m); }
    Vector operator*(int m) const { return *this * (double) m; }
    bool operator==(const Vector &o) const { return v[0] == o.v[0] && v[1] == o.v[1] && v[2] == o.v[2]; }
    bool operator!=(const Vector &o) const { return !(*this == o); }

    // Functions for vector logic
    Vector cross(const Vector &o) const {
        return Vector(v[1]*o.v[2] - v[2]*o.v[1], v[2]*o.v[0] - v[0]*o.v[2], v[0]*o.v[1] - v[1]*o.v[0]);
    }
    double norm() const { return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
    Vector normalize() const { return *this * (1 / norm()); }

    // Getter functions
    double operator[](int i) const { return v[i]; }
    boost::python::list get_item_slice(const boost::python::slice &slice) const;

    // Other stuff - len, iter, repr
    int size() const { return 3; }
    const double* cbegin() const { return v.cbegin(); }
    const double* cend() const { return v.cend(); }
    std::string to_string() const;
};

// LHS operators of vectors
Vector operator*(double m, const Vector& o) { return o * m; }
Vector operator*(int m, const Vector &o) { return o * m; }

struct Ray {
    Vector a, b;
    Ray(const Vector &a, const Vector &b) : a(a), b(b) {}
};

struct Triangle {
    const Vector a, b, c;
    Triangle(const Vector &a, const Vector &b, const Vector &c) : a(a), b(b), c(c) {}
};

struct FakeRect {
    const Triangle t1, t2;
    FakeRect(const Vector &offset, const Vector &vec_x, const Vector &vec_y)
        : t1(offset, offset+vec_x, offset+vec_x+vec_y), t2(offset, offset+vec_y, offset+vec_x+vec_y) {}
};

// Smart functions
boost::python::tuple intersect_ray_triangle(const Ray &r, const Triangle &t);
boost::python::list rotate_3d(
    const Vector &rot_axis, double rot_angle, const boost::python::list &vectors);
boost::python::list create_grid(
    int width, int height, const Vector &vec_x, const Vector &vec_y, const Vector &offset);
boost::python::tuple intersect_ray_fakerect(const Ray &r, const FakeRect &fr);
boost::python::tuple intersect_ray_scene(const Ray &r, const boost::python::list &triangles);
bool intersects_scene(const Ray &r, const boost::python::list &triangles, int ignore_idx);
boost::python::tuple raycast(
    const boost::python::list &grid,
    const Vector &source,
    const boost::python::list &scene_triangles,
    const Vector &drain,
    const FakeRect &drain_rect,
    const boost::python::list background_triangles);
boost::python::list convert_coordinates_2d(
    const boost::python::list &coordinates,
    const Vector &drain_rect_origin,
    const Vector &scaled_basis_x,
    const Vector &scaled_basis_y,
    double rot_angle);
boost::python::list depth_to_scene(
        const boost::python::list &grid,
        const Vector &source,
        const boost::python::list &scene_triangles);

}

// boost export of the stuff
using namespace boost::python;

BOOST_PYTHON_MODULE(geometry_cpp) {
    class_<ster::Vector>("Vector", init<double, double, double>())
        .def(self * double())
        .def(double() * self)
        .def(self / double())
        .def(self + self)
        .def(self - self)
        .def(self * int())
        .def(int() * self)
        .def(self * self)
        .def(self == self)
        .def("__repr__", &ster::Vector::to_string)
        .def("__getitem__", &ster::Vector::operator[])
        .def("__getitem__", &ster::Vector::get_item_slice)
        .def("__iter__", range(&ster::Vector::cbegin, &ster::Vector::cend))
        .def("__len__", &ster::Vector::size)
        .def("norm", &ster::Vector::norm)
        .def("normalize", &ster::Vector::normalize)
        .def("cross", &ster::Vector::cross);
    class_<ster::Ray>("Ray", init<ster::Vector, ster::Vector>());
    class_<ster::Triangle>("Triangle", init<ster::Vector, ster::Vector, ster::Vector>())
        .def_readonly("a", &ster::Triangle::a)
        .def_readonly("b", &ster::Triangle::b)
        .def_readonly("c", &ster::Triangle::c);
    class_<ster::FakeRect>("FakeRect", init<ster::Vector, ster::Vector, ster::Vector>())
        .def_readonly("t1", &ster::FakeRect::t1)
        .def_readonly("t2", &ster::FakeRect::t2);
    def("intersect_ray_triangle", ster::intersect_ray_triangle);
    def("rotate_3d", ster::rotate_3d);
    def("create_grid", ster::create_grid);
    def("intersect_ray_fakerect", ster::intersect_ray_fakerect);
    def("intersect_ray_scene", ster::intersect_ray_scene);
    def("intersects_scene", ster::intersects_scene);
    def("raycast", ster::raycast);
    def("convert_coordinates_2d", ster::convert_coordinates_2d);
    def("depth_to_scene", ster::depth_to_scene);
}

#endif
