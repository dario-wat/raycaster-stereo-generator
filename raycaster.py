import sys
import time
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rcutils
from geometry import rayIntersectTriangle, rotate3d

from geometry_cpp import intersect_ray_triangle, Vector, Ray, Triangle

def plotP(ax, p):
    ax.plot([p[0]], [p[1]], 'o', zs=[p[2]])

def plotL(ax, a, b):
    ax.plot([a[0], b[0]], [a[1], b[1]], '-', zs=[a[2], b[2]])

def raycaster(source, focalVec, vertVec, horizVec, rotAngle, gridSize, vertices, triangles):
    pass
    # focalVec = focalVec / np.linalg.norm(focalVec)
    # s = np.sin(rotAngle / 2.)
    # q0, q1, q2, q3 = np.cos(rotAngle / 2.), s*focalVec[0], s*focalVec[1], s*focalVec[2]
    # Q = np.array(
    #     [[q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
    #     [2*q1*q2 + 2*q0*q3, q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*q2*q3 - 2*q0*q1],
    #     [2*q1*q3 - 2*q0*q2, 2*q3*q2 + 2*q0*q1, q0*q0 - q1*q1 - q2*q2 + q3*q3]])
    # tHoriz = horizVec #/ np.linalg.norm(horizVec)#- source
    # tVert = vertVec #/np.linalg.norm(vertVec)#- source
    # rHoriz = np.dot(Q, tHoriz)
    # rVert = np.dot(Q, tVert)
    # rHorizVec = rHoriz #+ source
    # rVertVec = rVert #+ source
    # return rHorizVec, rVertVec

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print 'Need file name'
        exit()

    vertices, triangles = rcutils.readFace(sys.argv[1])
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    collection = rcutils.createCollection(vertices, triangles)
    ax.add_collection3d(Poly3DCollection(collection, facecolors='0.75', edgecolors='k'))

    widthPxl = 32
    heightPxl = 24
    widthChip = 1.6
    heightChip = 1.2
    focalLength = 1.5

    veca = np.array([1.0, 0.0, 0.0]) * widthChip / widthPxl
    vecb = np.array([0.0, 1.0, 0.0]) * heightChip / heightPxl
    focVec = np.array([0., 0., -1.]) * focalLength
    source = np.array([0., 0., 3.])

    plotL(ax, source + focVec, source + focVec + veca*10)
    plotL(ax, source + focVec, source + focVec + vecb*10)

    start = time.time()
    # veca, vecb = raycaster(source, focVec, vecb, veca, np.pi/4, None, None, None)
    veca, vecb = rotate3d(focVec, np.pi/4, [veca, vecb])
    print time.time() - start
    veca = veca / np.linalg.norm(veca) * widthChip / widthPxl
    vecb = vecb / np.linalg.norm(vecb) * heightChip / heightPxl
    plotL(ax, source + focVec, source + focVec + veca*10)
    plotL(ax, source + focVec, source + focVec + vecb*10)
    offset = source + focVec - veca * widthPxl / 2. - vecb * heightPxl / 2.

    plotP(ax, source)
    plotL(ax, source, source + focVec)

    start = time.time()
    grid = np.zeros((widthPxl*heightPxl, 3))
    for i in xrange(widthPxl):
        for j in xrange(heightPxl):
            grid[i*heightPxl+j] = i*veca + j*vecb + offset
    ax.plot(grid[:,0], grid[:,1], 'r.', zs=grid[:,2])
    print 'Grid generate', time.time() - start
    
    start = time.time()
    for po in grid:
        # print p
        ray = np.vstack([source, po])
        # ax.plot(ray[:,0], ray[:,1], zs=ray[:,2])
        ray_cpp = Ray(Vector(*source), Vector(*po))
        for t in triangles:
            triangle_cpp = Triangle(Vector(*vertices[t[0]]), Vector(*vertices[t[1]]), Vector(*vertices[t[2]]))
            # f, p = rayIntersectTriangle(ray, vertices[t])
            f, p = intersect_ray_triangle(ray_cpp, triangle_cpp)
            if f == 1:
                # ax.add_collection3d(Poly3DCollection([vertices[t]], edgecolors='k'))
                # ax.plot([source[0], p.x], [source[1], p.y], zs=[source[2], p.z])
                ax.plot([p.x], [p.y], 'r.', zs=[p.z])
    
    print time.time() - start

    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    plt.show()

