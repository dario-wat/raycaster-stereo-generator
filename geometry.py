import numpy as np

# TODO delete in the next commit
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


def rotate3d(rotAxis, rotAngle, vectors):
    """Rotates vectors and points around the axis of rotation.
        rotAxis     - the axis of rotation (it goes through the origin and is normalized)
        rotAngle    - angle of rotation
        vectors     - a list of vectors to rotate or points that need to be translated around the
                      origin since the axis of rotation goes through origin and the rotation
                      would not work otherwise."""
    rotAxis = rotAxis / np.linalg.norm(rotAxis)
    s, c = np.sin(rotAngle / 2.), np.cos(rotAngle / 2.)
    q0, q1, q2, q3 = c, s*rotAxis[0], s*rotAxis[1], s*rotAxis[2]
    q00, q11, q22, q33 = q0*q0, q1*q1, q2*q2, q3*q3
    q01, q02, q03, q12, q13, q23 = q0*q1, q0*q2, q0*q3, q1*q2, q1*q3, q2*q3
    Q = np.array(
        [[q00 + q11 - q22 - q33,         2*(q12 - q03),         2*(q13 + q02)],
        [         2*(q12 + q03), q00 - q11 + q22 - q33,         2*(q23 - q01)],
        [         2*(q13 - q02),         2*(q23 + q01), q00 - q11 - q22 + q33]])
    return map(lambda v: np.dot(Q, v), vectors)