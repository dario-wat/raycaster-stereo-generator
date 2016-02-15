import sys
import time
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rcutils
# import geometry

from geometry_cpp import intersect_ray_triangle, rotate_3d, Vector, Ray, Triangle

def plotP(ax, p):
    ax.plot([p[0]], [p[1]], 'o', zs=[p[2]])

def plotL(ax, a, b):
    ax.plot([a[0], b[0]], [a[1], b[1]], '-', zs=[a[2], b[2]])

def plotT(ax, t):
    ax.add_collection3d(Poly3DCollection([[t.a, t.b, t.c]], facecolors='#ffaab4', edgecolors='r'))

def raycaster(source, focalVec, vertVec, horizVec, rotAngle, gridSize, vertices, triangles):
    pass

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print 'Need file name'
        exit()

    vertices, triangles = rcutils.readFace(sys.argv[1])
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    collection = rcutils.createCollection(vertices, triangles)
    ax.add_collection3d(Poly3DCollection(collection, facecolors='0.75', edgecolors='k'))

    widthPxl = 10
    heightPxl = 5
    widthChip = 1.6
    heightChip = 1.2
    focalLength = 1.5

    focVec = Vector(0., 0., -1.) * focalLength
    veca = Vector(1., 0., 0.) * (widthChip / widthPxl)
    vecb = Vector(0., 1., 0.) * (heightChip / heightPxl)
    source = Vector(0., 0., 3.)

    # veca, vecb = raycaster(source, focVec, vecb, veca, np.pi/4, None, None, None)
    veca, vecb = rotate_3d(focVec, np.pi/4, [veca, vecb])
    veca = veca / veca.norm() * widthChip / widthPxl
    vecb = vecb / vecb.norm() * heightChip / heightPxl
    offset = source + focVec - veca * widthPxl / 2. - vecb * heightPxl / 2.

    start = time.time()
    grid = np.zeros((widthPxl*heightPxl, 3))
    for i in xrange(widthPxl):
        for j in xrange(heightPxl):
            grid[i*heightPxl+j] = i*veca + j*vecb + offset
    print time.time() - start
    ax.plot(grid[:,0], grid[:,1], 'y.', zs=grid[:,2])
    # plotP(ax, source)
    # plotL(ax, source, source + focVec)
    # plotL(ax, source, source + focVec + veca * widthPxl / 2)
    # plotL(ax, source, source + focVec + vecb * heightPxl / 2)

    source2 = Vector(0., 1., 3.)
    focVec2 = Vector(0., 0., -1.) * focalLength
    # veca, vecb = rotate_3d(focVec2, np.pi/4, [veca, vecb])
    veca = veca / veca.norm() * widthChip / widthPxl
    vecb = vecb / vecb.norm() * heightChip / heightPxl
    offset = source2 + focVec2 - veca * widthPxl / 2. - vecb * heightPxl / 2.
    grid2 = np.zeros((widthPxl*heightPxl, 3))
    for i in xrange(widthPxl):
        for j in xrange(heightPxl):
            grid2[i*heightPxl+j] = list(i*veca + j*vecb + offset)
    ax.plot(grid2[:,0], grid2[:,1], 'b.', zs=grid2[:,2])
    # plotP(ax, source2)
    # plotL(ax, source2, source2 + focVec2)
    # plotL(ax, source2, source2 + focVec2 + veca * widthPxl / 2)
    # plotL(ax, source2, source2 + focVec2 + vecb * heightPxl / 2)
    
    # TODO something is wrong with this intersection
    tri1 = Triangle(offset, offset + veca*(widthPxl-1), offset + veca*(widthPxl-1) + vecb*(heightPxl-1))
    tri2 = Triangle(offset, offset + veca*(heightPxl-1), offset + veca*(widthPxl-1) + vecb*(heightPxl-1))
    plotT(ax, tri1)

    start = time.time()
    for po in grid:
        # print p
        # ray = np.vstack([list(source), po])
        # ax.plot(ray[:,0], ray[:,1], zs=ray[:,2])
        ray_cpp = Ray(Vector(*source), Vector(*po))
        for t in triangles:
            triangle_cpp = Triangle(Vector(*vertices[t[0]]), Vector(*vertices[t[1]]), Vector(*vertices[t[2]]))
            # f, p = rayIntersectTriangle(ray, vertices[t])
            f, p = intersect_ray_triangle(ray_cpp, triangle_cpp)
            if f == 1:
                # ax.add_collection3d(Poly3DCollection([vertices[t]], edgecolors='k'))
                plotL(ax, source2, p)
                intr = Ray(source2, p)
                ax.plot([source[0], p[0]], [source[1], p[1]], 'r-', zs=[source[2], p[2]])
                ax.plot([p[0]], [p[1]], 'r.', zs=[p[2]])

                f, p = intersect_ray_triangle(intr, tri1)
                if f == 1:
                    plotP(ax, p)
                f, p = intersect_ray_triangle(intr, tri2)
                if f == 1:
                    plotP(ax, p)
    
    print time.time() - start

    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    plt.show()

