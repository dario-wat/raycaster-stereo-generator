import numpy as np
import math
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from geometry_cpp import intersects_scene, intersect_ray_scene, Ray, intersect_ray_fakerect, rotate_3d

def plotP(ax, p, marker='.', color=None):
    """Plot point"""
    ax.plot([p[0]], [p[1]], marker=marker, zs=[p[2]], color=color)

def plotL(ax, a, b, color=None):
    """Plot line"""
    ax.plot([a[0], b[0]], [a[1], b[1]], '-', zs=[a[2], b[2]], color=color)

# TODO should make support for several, and color stuff too
def plotT(ax, t):
    """Plot triangle"""
    ax.add_collection3d(Poly3DCollection([[t.a, t.b, t.c]], facecolors='#ffaab4', edgecolors='r'))

def plotG(ax, grid, color=None):
    """Plots a grid of points"""
    xs, ys, zs = zip(*grid)
    ax.plot(xs, ys, '.', zs=zs, color=color)

def plotImVs(ax, point, focvec, baseX, baseY, widthPxl, heightPxl, color=[None, None, None, None]):
    plotP(ax, point, color[0])
    plotL(ax, point, point + focvec, color[1])
    plotL(ax, point, point + focvec + baseX * (widthPxl-1) / 2., color[2])
    plotL(ax, point, point + focvec + baseY * (heightPxl-1) / 2., color[3])

def rayCaster(grid, source, sceneTriangles, drain, drainRect, axp):
    """Seems complicated, read raycast in cpp. This one is still
    here so that some drawings can be made."""
    coords = []
    for gridPoint in grid:
        ray = Ray(source, gridPoint)
        intersFound, intersPoint, triangleIdx = intersect_ray_scene(ray, sceneTriangles)
        if intersFound != 1:  # this should always be false
            coords.append(None)
            continue
        
        # plotL(axp, drain, intersPoint, color='r')
        plotL(axp, source, intersPoint, color='b')
        # plotP(axp, intersPoint, color='r')

        backRay = Ray(intersPoint, drain)
        intersBackFound, intersBackPoint = intersect_ray_fakerect(backRay, drainRect)
        if intersBackFound != 1 or intersects_scene(backRay, sceneTriangles, triangleIdx):
            coords.append(None)
            continue
        
        coords.append(intersBackPoint)
        plotP(axp, intersBackPoint, 'o')
    return coords

def testCoords(coords1, coords2):
    """Compares 2 lists of coordinates for equality."""
    for a, b in zip(coords1, coords2):
        if a is None or b is None:
            if a != b:
                return False
        elif abs(a[0] - b[0]) > 1e-12 or abs(a[1] - b[1]) > 1e-12:
            return False
    return True

def createGridImage(width, height, gap):
    img = np.ones((height, width), dtype=np.uint8)*255
    for i in xrange(gap, width, gap):
        img[:,i] = 0
    for i in xrange(gap, height, gap):
        img[i,:] = 0
    return img

N8 = [(1,0), (-1,0), (0,1), (0,-1), (-1,-1), (-1,1), (1,-1), (1,1)]

# Tricky function, might mess up disparity map. Check the results before saving it
def fillDisparityMap(disparityMap, threshold=7):
    h, w = disparityMap.shape
    disparityMapCopy = np.copy(disparityMap)
    mind = disparityMap[disparityMap >= 0.0].min()
    for i in xrange(h):
        for j in xrange(w):
            if disparityMap[i,j] >= 0.0:
                continue

            count = 0
            s = 0.0
            for dx, dy in N8:
                ii = i+dx
                jj = j+dy
                if ii < 0 or jj < 0 or ii >= h or jj >= w or disparityMap[ii,jj] < 0.0:
                    continue
                s += disparityMap[ii,jj]
                count += 1

            if count >= threshold:
                disparityMapCopy[i,j] = s / count
            else:
                disparityMapCopy[i,j] = mind
    return disparityMapCopy
