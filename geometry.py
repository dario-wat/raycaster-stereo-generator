import numpy as np
from math import sqrt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from geometry_cpp import intersects_scene, intersect_ray_scene, Ray, intersect_ray_fakerect

def plotP(ax, p, marker='.', color=None):
    ax.plot([p[0]], [p[1]], marker=marker, zs=[p[2]], color=color)

def plotL(ax, a, b, color=None):
    ax.plot([a[0], b[0]], [a[1], b[1]], '-', zs=[a[2], b[2]], color=color)

def plotT(ax, t):
    ax.add_collection3d(Poly3DCollection([[t.a, t.b, t.c]], facecolors='#ffaab4', edgecolors='r'))

def rayCaster(grid, source, sceneTriangles, drain, drainRect, veca, vecb, ax):
    coords = []
    for gridPoint in grid:
        ray = Ray(source, gridPoint)
        intersFound, intersPoint, triangleIdx = intersect_ray_scene(ray, sceneTriangles)
        if intersFound != 1:  # this should always be false
            coords.append(None)
            continue
        
        # plotL(ax, drain, intersPoint, color='r')
        # plotL(ax, source, intersPoint, color='b')
        # plotP(ax, intersPoint, color='r')

        backRay = Ray(intersPoint, drain)
        intersBackFound, intersBackPoint = intersect_ray_fakerect(backRay, drainRect)
        if intersBackFound != 1 or intersects_scene(backRay, sceneTriangles, triangleIdx):
            coords.append(None)
            continue
        
        # this should go out
        v1 = intersBackPoint - drainRect.origin()
        vecx = veca / veca.norm()
        vecy = vecb / vecb.norm()
        cx, cy, cz = v1
        ax0, ay, az = vecx
        bx, by, bz = vecy

        m = (cy - by * cx/bx) / (ay - by*ax0/bx)
        n = (cy - ay*cx/ax0) / (by - ay*bx/ax0)
        coords.append((m, n))
        
        # plotL(ax, offset, offset + n*vecy)
        # plotL(ax, offset + n*vecy, offset + m*vecx + n*vecy)
        # plotL(ax, offset, offset + veca)
        # plotL(ax, offset, offset + vecb)
        # print m / veca.norm(), n / vecb.norm()
        plotP(ax, intersBackPoint)
    return coords