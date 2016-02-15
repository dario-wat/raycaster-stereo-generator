import numpy as np


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