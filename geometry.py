import numpy as np
from math import sqrt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from geometry_cpp import intersect_ray_triangle, intersects_scene, intersect_ray_scene, Vector, Ray, Triangle, intersect_ray_fakerect

def plotP(ax, p):
    ax.plot([p[0]], [p[1]], 'o', zs=[p[2]])

def plotL(ax, a, b):
    ax.plot([a[0], b[0]], [a[1], b[1]], '-', zs=[a[2], b[2]])

def plotT(ax, t):
    ax.add_collection3d(Poly3DCollection([[t.a, t.b, t.c]], facecolors='#ffaab4', edgecolors='r'))

def rayCaster(grid, source, sceneTriangles, drain, drainRect, veca, vecb, ax):
    coords = []
    for po in grid:
        ray_cpp = Ray(Vector(*source), Vector(*po))

        f, p2, idx = intersect_ray_scene(ray_cpp, sceneTriangles)
        if f == 1:  # this should always be true
            # plotL(ax, drain, p2)

            plotL(ax, source, p2)
            # ax.plot([source[0], p2[0]], [source[1], p2[1]], 'r-', zs=[source[2], p2[2]])
            ax.plot([p2[0]], [p2[1]], 'r.', zs=[p2[2]])

            intr = Ray(p2, drain)
            f, p = intersect_ray_fakerect(intr, drainRect)
            if f == 1:
                if intersects_scene(intr, sceneTriangles, idx):
                    coords.append(None)
                    continue
                
                # print cnt
                v1 = p - drainRect.origin()
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
                # print n / veca.norm(), m / vecb.norm()
                # print cx, bx, ax0
                # print m / veca.norm(), n / vecb.norm()
                # plotP(ax, p)
            else:
                coords.append(None)
        else:
            coords.append(None)
    return coords