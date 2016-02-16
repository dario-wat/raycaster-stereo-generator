import numpy as np
from math import sqrt
from geometry_cpp import intersect_ray_triangle, intersects_scene, intersect_ray_scene, Vector, Ray, Triangle, intersect_ray_fakerect

def plotP(ax, p):
    ax.plot([p[0]], [p[1]], 'o', zs=[p[2]])

def plotL(ax, a, b):
    ax.plot([a[0], b[0]], [a[1], b[1]], '-', zs=[a[2], b[2]])

def rayCaster(grid, source, triangles_cpp, vertices, source2, fr, offset, veca, vecb, ax):
    cnt = 0
    for po in grid:
        # ray = np.vstack([list(source), po])
        # ax.plot(ray[:,0], ray[:,1], zs=ray[:,2])
        ray_cpp = Ray(Vector(*source), Vector(*po))

        f, p, _ = intersect_ray_scene(ray_cpp, triangles_cpp)
        if f == 1:
            # plotL(ax, source2, p)
            # ax.plot([source[0], p[0]], [source[1], p[1]], 'r-', zs=[source[2], p[2]])
            # ax.plot([p[0]], [p[1]], 'r.', zs=[p[2]])

            intr = Ray(source2, p)
            f, p = intersect_ray_fakerect(intr, fr)
            if f == 1:
                if intersects_scene(intr, triangles_cpp):
                    cnt += 1
                print cnt
                v1 = p - offset
                vecx = veca / veca.norm()
                vecy = vecb / vecb.norm()
                cx, cy, cz = v1
                ax0, ay, az = vecx
                bx, by, bz = vecy

                m = (cy - by * cx/bx) / (ay - by*ax0/bx)
                n = (cy - ay*cx/ax0) / (by - ay*bx/ax0)
                
                # plotL(ax, offset, offset + n*vecy)
                # plotL(ax, offset + n*vecy, offset + m*vecx + n*vecy)
                # plotL(ax, offset, offset + veca)
                # plotL(ax, offset, offset + vecb)
                # print n / veca.norm(), m / vecb.norm()
                # print cx, bx, ax0
                # print m / veca.norm(), n / vecb.norm()
                # plotP(ax, p)
