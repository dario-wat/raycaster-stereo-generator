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

DEEP_PINK = [147, 20, 255]
N8 = np.array([[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]])

def transformVirtual(widthPxl, heightPxl, backCoords, origImg):
    """Transforms real image into virtual using the previously raycasted coordinates.
    Bilinear interpolation works with the assumption that Y values are already aligned."""
    virtualImPxls = np.mgrid[0:widthPxl, 0:heightPxl].reshape(2, widthPxl*heightPxl).T
    originalImPxls = np.array(backCoords)

    virtualImg = np.zeros((heightPxl, widthPxl), np.uint8)
    disparityMap = np.zeros((heightPxl, widthPxl), np.float32)
    virtualVisualImg = np.zeros((heightPxl, widthPxl, 3), np.uint8)
    occlusionMask = np.zeros((heightPxl, widthPxl), np.uint8)
    virtualVisualImg[:,:] = DEEP_PINK
    
    for i in xrange(widthPxl*heightPxl):
        x, y = virtualImPxls[i]
        if originalImPxls[i] is None:
            occlusionMask[y,x] = 255
            continue
        j, k = originalImPxls[i]
        disparityMap[y,x] = abs(x-j)
        y, k = round(y), round(k)
        dj = abs(j-math.trunc(j))
        virtualImg[y,x] = int((1-dj)*origImg[k,j,0] + dj*origImg[k,j+1,0])
        # virtualImg[y,x] = origImg[k,j,0]
        virtualVisualImg[y,x] = origImg[k,j]

    return virtualImg, disparityMap, virtualVisualImg, occlusionMask

def missingInterpolation(imgOrig, occlusionMask, nIter=1):
    """Fills in occluded pixels by the mean value of neighbors."""
    img = np.copy(imgOrig)
    height, width = occlusionMask.shape
    occlusionMaskCopy = np.copy(occlusionMask)
    for _ in xrange(nIter):
        imgCopy = np.copy(img)
        interCoords = np.array(occlusionMaskCopy.nonzero()).T
        occlusionMaskCopy2 = np.copy(occlusionMaskCopy)
        for y, x in interCoords:
            sumFilter = 0
            count = 0
            for dy, dx in N8:
                xn, yn = x+dx, y+dy
                if xn < 0 or yn < 0 or xn >= width or yn >= height or occlusionMaskCopy2[yn, xn] > 0:
                    continue
                sumFilter += int(imgCopy[yn,xn])
                count += 1
            if count != 0:
                occlusionMaskCopy[y,x] = 0
                img[y,x] = sumFilter / count    # this integer division is fine
    return img

def createGridImage(width, height, gap):
    img = np.ones((height, width), dtype=np.uint8)*255
    for i in xrange(gap, width, gap):
        img[:,i] = 0
    for i in xrange(gap, height, gap):
        img[i,:] = 0
    return img