#include "geometry.h"

#include <cmath>

// const ster::Vector ster::Vector::ZERO = ster::Vector(0.0, 0.0, 0.0);

int ster::intersect_ray_triangle(
        const ster::Ray &ray, const ster::Triangle &triangle, ster::Vector &p) {
    // // triangle vectors
    // ster::Vector u = triangle.b - triangle.a;
    // ster::Vector v = triangle.c - triangle.a;
    // ster::Vector n = u.cross(v);
    // if (n == ster::Vector::ZERO) {
    //     return -1;                      // degenerate triangle
    // }

    // // ray vectors
    // ster::Vector d = ray.b - ray.a;         // direction
    // ster::Vector w0 = ray.a - triangle.a;
    // double a = n * w0;
    // double b = n * d;
    // if (fabs(b) < 1e-6) {               // ray is parallel to the triangle
    //     return 2;
    // }

    // double r = a / b;
    // if (r < 0.0) {                      // no intersection, ray goes away from the triangle
    //     return 0;
    // }

    // // intersection of a ray and a plane
    // ster::Vector intersection = ray.a + d*r;

    // // checking if intersection is inside triangle
    // double uu = u * u;
    // double uv = u * v;
    // double vv = v * v;
    // ster::Vector w = intersection - triangle.a;
    // double wu = w * u;
    // double wv = w * v;
    // double D = uv*uv - uu*vv;

    // // test parametric coordinates
    // double s = (uv*wv - vv*wu) / D;
    // if (s < 0.0 || s > 1.0) {       // intersection is outside of the triangle
    //     return 0;
    // }

    // double t = (uv*wu - uu*wv) / D;
    // if (t < 0.0 || s + t > 1.0) {   // intersection is outside of the triangle
    //     return 0;
    // }

    // p.x = intersection.x;
    // p.y = intersection.y;
    // p.z = intersection.z;
    return 1;
}
