import numpy as np

def rayIntersectTriangle(ray, triangle):
    """Intersection of a ray and a triangle. Ray is a numpy of 2 points,
        triangle is a numpy of 3 points."""
    # triangle vectors
    u = triangle[1] - triangle[0]
    v = triangle[2] - triangle[0]
    n = np.cross(u, v)
    if (n == 0.0).all():
        return -1, None     # degenerate triangle

    # ray vectors   
    d = ray[1] - ray[0]         # direction
    w0 = ray[0] - triangle[0]
    a = -np.dot(n, w0)
    b = np.dot(n, d)
    if np.fabs(b) < 1e-6:   # ray is parallel to the triangle
        return (2, None) if a == 0 else (0, None)

    r = a / b
    if r < 0.0:             # no intersection, ray goes away from triangle
        return 0, None

    # intersection of a ray and a plane
    intersection = ray[0] + r*d
    
    # checking if intersection is inside the triangle
    uu = np.dot(u, u)
    uv = np.dot(u, v)
    vv = np.dot(v, v)
    w = intersection - triangle[0]
    wu = np.dot(w, u)
    wv = np.dot(w, v)
    D = uv*uv - uu*vv

    # test parametric coordinates
    s = (uv*wv - vv*wu) / D
    if s < 0.0 or s > 1.0:      # intersection is outside of the triangle
        return 0, None

    t = (uv*wu - uu*wv) / D
    if t < 0.0 or s + t > 1.0:  # intersection is outside of the triangle
        return 0, None

    return 1, intersection
