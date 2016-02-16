import sys
import time
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rcutils
from geometry import plotT, plotP, plotL, rayCaster, testCoords

from geometry_cpp import convert_coordinates_2d, rotate_3d, Vector, Triangle, FakeRect, create_grid, raycast

def raycaster(source, focalVec, vertVec, horizVec, rotAngle, gridSize, vertices, triangles):
    pass

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print 'Need file name'
        exit()

    vertices, triangles = rcutils.readFace(sys.argv[1])
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    vertices = np.vstack((vertices, [[2, 2, -0.328], [2, -2, -0.328], [-2, -2, -0.328], [-2, 2, -0.328]]))
    i = len(vertices)
    triangles = np.vstack((triangles, [[i-4, i-3, i-2], [i-4, i-2, i-1]]))
    collection = rcutils.createCollection(vertices, triangles)
    ax.add_collection3d(Poly3DCollection(collection, facecolors='0.75', edgecolors='k'))

    widthPxl = 32
    heightPxl = 24
    widthChip = 1.6
    heightChip = 1.2
    focalLength = 1.5
    rotAngle = np.pi / 4

    focVec = Vector(0., 0., -1.) * focalLength
    veca = Vector(1., 0., 0.) * (widthChip / widthPxl)
    vecb = Vector(0., 1., 0.) * (heightChip / heightPxl)
    source = Vector(0., 0., 3.)

    veca, vecb = rotate_3d(focVec, rotAngle, [veca, vecb])
    veca = veca / veca.norm() * widthChip / widthPxl
    vecb = vecb / vecb.norm() * heightChip / heightPxl
    offset = source + focVec - veca * (widthPxl-1) / 2. - vecb * (heightPxl-1) / 2.

    grid = create_grid(widthPxl, heightPxl, veca, vecb, offset)
    xs, ys, zs = zip(*grid)
    # ax.plot(xs, ys, 'y.', zs=zs)

    # plotP(ax, source)
    # plotL(ax, source, source + focVec)
    # plotL(ax, source, source + focVec + veca * widthPxl / 2)
    # plotL(ax, source, source + focVec + vecb * heightPxl / 2)

    source2 = Vector(1.2, 1.2, 3.)
    focVec2 = Vector(0., 0., -1.) * focalLength
    # veca, vecb = rotate_3d(focVec2, rotAngle, [veca, vecb])
    veca = veca / veca.norm() * widthChip / widthPxl
    vecb = vecb / vecb.norm() * heightChip / heightPxl
    offset = source2 + focVec2 - veca * widthPxl / 2. - vecb * heightPxl / 2.
    

    # drain grid plotting
    # grid2 = create_grid(widthPxl, heightPxl, veca, vecb, offset)
    # xs, ys, zs = zip(*grid2)
    # ax.plot(xs, ys, 'b.', zs=zs)
    
    # plotting vectors for drain grid
    # plotP(ax, source2)
    # plotL(ax, source2, source2 + focVec2)
    # plotL(ax, source2, source2 + focVec2 + veca * widthPxl / 2)
    # plotL(ax, source2, source2 + focVec2 + vecb * heightPxl / 2)
    
    # drain grid rectangle
    drainRectangle = FakeRect(offset, veca*(widthPxl-1), vecb*(heightPxl-1))
    plotT(ax, drainRectangle.t1)
    plotT(ax, drainRectangle.t2)

    triangles_cpp = []
    for t in triangles:
        triangles_cpp.append(Triangle(Vector(*vertices[t[0]]), Vector(*vertices[t[1]]), Vector(*vertices[t[2]])))

    # The actual raycasting
    start = time.time()
    coords = rayCaster(grid, source, triangles_cpp, source2, drainRectangle, veca, vecb, ax)
    coords2 = raycast(grid, source, triangles_cpp, source2, drainRectangle)
    # print len(coords), len(grid), widthPxl*heightPxl, len(coords2)

    coords3 = convert_coordinates_2d(coords2, drainRectangle.origin(), veca, vecb)
    print time.time() - start

    # print testCoords(coords, coords3)

    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    plt.show()
