import sys
import time
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rcutils
from geometry import rayIntersectTriangle

from geometry_cpp import intersect_ray_triangle, Vector, Ray, Triangle

def raycaster(source, focalVec, rotation, gridSize, vertices, triangles):
    pass

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print 'Need file name'
        exit()

    vertices, triangles = rcutils.readFace(sys.argv[1])
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    xsize = 32
    ysize = 24

    veca = np.array([1.0, 0.0, 0.0]) * 2 / xsize
    vecb = np.array([0.0, 1.0, 0.0]) * 2 / ysize
    offset = [-1.0, -1.0, 1.5]
    grid = np.zeros((xsize*ysize, 3))
    for i in xrange(xsize):
        for j in xrange(ysize):
            grid[i*ysize+j] = i*veca + j*vecb + offset
    ax.plot(grid[:,0], grid[:,1], 'r.', zs=grid[:,2])
    
    source = np.array([0., 0., 3.])

    start = time.time()
    for po in grid:
        # print p
        ray = np.vstack([source, po])
        ax.plot(ray[:,0], ray[:,1], zs=ray[:,2])
        ray_cpp = Ray(Vector(source[0], source[1], source[2]), Vector(po[0], po[1], po[2]))
        for t in triangles:
            
            triangle_cpp = Triangle(Vector(*vertices[t[0]]), Vector(*vertices[t[1]]), Vector(*vertices[t[2]]))
            # print intersect_ray_triangle(Ray(Vector()))
            f, p = intersect_ray_triangle(ray_cpp, triangle_cpp)
            if f == 1:
                # ax.add_collection3d(Poly3DCollection([vertices[t]], edgecolors='k'))
                ax.plot([source[0], p.x], [source[1], p.y], zs=[source[2], p.z])
                ax.plot([p.x], [p.y], 'ro', zs=[p.z])
    
    print time.time() - start
    collection = rcutils.createCollection(vertices, triangles)
    ax.add_collection3d(Poly3DCollection(collection, facecolors='0.75', edgecolors='k'))
    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    plt.show()

