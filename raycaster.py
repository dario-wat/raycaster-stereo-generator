import sys
import time
import math

import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rcutils
from geometry import plotT, plotP, plotL, plotG, plotImVs, rayCaster, testCoords

from geometry_cpp import convert_coordinates_2d, rotate_3d, Vector, Triangle, FakeRect, \
    create_grid, raycast, depth_to_scene

# Some camera parameters
# focal length: 5.15 mm
# sensor width: 2600 pixels
# sensor height: 1952 pixels
# pixel width: 0.0014 mm
# pixel height: 0.0014 mm
# subsampled height: 488 px
# subsampled width: 656 px
# focal length in px = 5.15/(0.0014*0.5*(2600/656+1952/488)) = 924

# Constants
ZEROVEC = Vector(0., 0., 0.)
N8 = np.array([[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]])
DEEP_PINK = [147, 20, 255]

# Camera parameters
Z_HEIGHT = 3.
X_OFF = -0.3
Y_OFF = -0.3
B = 0.25
TILT = 0.0

# Default vector and camera position that is later used to rotate and place image plane
# easier in 3D space. Includes perpendicular vectors for rotating the side image vectors.
defaultVecX = Vector(1., 0., 0.)
defaultVecY = Vector(0., 1., 0.)
defaultVecDir = Vector(0., 0., -1.)
perpX = defaultVecDir.cross(defaultVecX)
perpY = defaultVecDir.cross(defaultVecY)

def positionCamera(point, direction, rotAngle):
    """Highly globally dependent function. Very complicated geometry positioning"""
    perpendicularAxis = direction.cross(defaultVecDir)
    angle = math.atan2(perpendicularAxis.norm(), direction*defaultVecDir)
    focalVec, vecxx, vecyy = (focVec, imVecX, imVecY) if defaultVecDir.cross(direction) == ZEROVEC \
        else rotate_3d(perpendicularAxis, angle, [focVec, imVecX, imVecY])
    vecxx, vecyy = rotate_3d(focalVec, rotAngle, [vecxx, vecyy])
    vecbx = (focalVec - vecxx).normalize() * widthP
    vecby = (focalVec - vecyy).normalize() * heightP
    origin = point + focalVec - vecbx * (widthPxl-1) / 2. - vecby * (heightPxl-1) / 2.
    return focalVec, vecbx, vecby, origin

def transformVirtual(widthPxl, heightPxl, backCoords, origImg):
    """Transforms real image into virtual using the previously raycasted coordinates.
    Bilinear interpolation works with the assumption that Y values are already proper."""
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
        dj = abs(j-math.trunc(j))
        # print (1-dj)*origImg[k,j,0], dj*origImg[k,j+1,0]
        # virtualImg[y,x] = int((1-dj)*origImg[k,j,0] + dj*origImg[k,j+1,0])
        # print origImg[k,j,0], int((1-dj)*origImg[k,j,0] + dj*origImg[k,j+1,0])
        virtualImg[y,x] = origImg[k,j,0]
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

if __name__ == '__main__':
    if len(sys.argv) <= 2:
        print 'Need file name'
        exit()

    # Read up the file
    vertices, triangles = rcutils.readFace(sys.argv[1])
    
    # Create plotter and add the scene to it
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    collection = rcutils.createCollection(vertices, triangles)
    ax.add_collection3d(Poly3DCollection(collection, facecolors='0.75', edgecolors='k'))

    # Read image and update parameters
    origImg = cv2.imread(sys.argv[2])

    # Intrinsic camera parameters
    widthPxl = origImg.shape[1]
    heightPxl = origImg.shape[0]
    widthChip = 0.0000014 * 2600 * 1.8
    heightChip = 0.0000014 * 1952 * 1.8
    focalLength = 0.00515
    widthC = widthChip * (widthPxl-1) / widthPxl
    heightC = heightChip * (heightPxl-1) / heightPxl
    widthP = widthChip / widthPxl
    heightP = heightChip / heightPxl

    # Intrinsic calculations that do not depend on camera position
    # Angle between the focal vector and vector that touches the half point of the side
    # of the image. Length of the said vector and the vector itself.
    angleX = math.atan(widthC/2./focalLength)
    angleY = math.atan(heightC/2./focalLength)
    lenX = math.sqrt(focalLength*focalLength + widthC*widthC/4.)
    lenY = math.sqrt(focalLength*focalLength + heightC*heightC/4.)
    imVecX = rotate_3d(perpX, angleX, [defaultVecDir])[0].normalize() * lenX
    imVecY = rotate_3d(perpY, angleY, [defaultVecDir])[0].normalize() * lenY
    focVec = defaultVecDir * focalLength

    # Source camera
    rotAngleS = 0
    source = Vector(X_OFF, Y_OFF, Z_HEIGHT)
    cameraDirS = Vector(-TILT, 0., -1.).normalize()
    focVecS, vecbxs, vecbys, originS = positionCamera(source, cameraDirS, rotAngleS)
    # plotImVs(ax, source, focVecS*1000, vecbxs*1000, vecbys*1000, widthPxl, heightPxl)
    plotL(ax, source, source + focVecS*1000, 'r')
    
    grid = create_grid(widthPxl, heightPxl, vecbxs, vecbys, originS)
    # plotG(ax, grid, 'r')

    # Drain camera
    rotAngleD = 0
    drain = Vector(X_OFF + B, Y_OFF, Z_HEIGHT)
    cameraDirD = Vector(TILT, 0., -1.).normalize()
    focVecD, vecbxd, vecbyd, originD = positionCamera(drain, cameraDirD, rotAngleD)
    # plotImVs(ax, drain, focVecD*1000, vecbxd*1000, vecbyd*1000, widthPxl, heightPxl)
    plotL(ax, drain, drain + focVecD*1000, 'b')
    
    # Drain grid plotting
    grid2 = create_grid(widthPxl, heightPxl, vecbxd, vecbyd, originD)
    # plotG(ax, grid2, 'b')
    
    # drain grid rectangle
    drainRectangle = FakeRect(originD, vecbxd*(widthPxl-1), vecbyd*(heightPxl-1))
    # plotT(ax, drainRectangle.t1)
    # plotT(ax, drainRectangle.t2)

    # Create triangles to be used for intersections
    triangles_cpp = []
    for t in triangles:
        triangles_cpp.append(
            Triangle(Vector(*vertices[t[0]]), Vector(*vertices[t[1]]), Vector(*vertices[t[2]])))

    # The actual raycasting
    start = time.time()
    # coords2 = rayCaster(grid, source, triangles_cpp, drain, drainRectangle, ax)
    coords, depthS, foreground = raycast(grid, source, triangles_cpp, drain, drainRectangle, [184, 185])    # HACK
    print 'Raycast time:', time.time() - start
    resultCoords = convert_coordinates_2d(coords, originD, vecbxd, vecbyd, rotAngleD)
    virtualImg, disparityMap, virtualVisual, occlusionMask = \
        transformVirtual(widthPxl, heightPxl, resultCoords, origImg)
    virtualImgFilled = missingInterpolation(virtualImg, occlusionMask, 1)
    print 'Time:', time.time() - start
    
    # Visualizing everythig
    cv2.imshow('Right image (real)', origImg)
    cv2.imshow('Left image (virtual)', virtualImg)
    # cv2.imshow('Occlusion mask', occlusionMask)
    cv2.imshow('Virtual with pink occlusions', virtualVisual)
    # cv2.imshow('Virtual image non-occluded', virtualImgFilled)

    # depthS2d = np.array(depthS).reshape([widthPxl, heightPxl]).T
    # cv2.imshow('Depth - virtual image', 1. - depthS2d / depthS2d.flatten().max())

    foreground = np.array(foreground, dtype=np.uint8).reshape([widthPxl, heightPxl]).T * 255
    # cv2.imshow('Foreground', foreground)

    cv2.imshow('Intersect face', virtualImg | foreground)

    # depth to drain, not useful
    # depthD = depth_to_scene(grid2, drain, triangles_cpp)
    # depthD2d = np.array(depthD).reshape([widthPxl, heightPxl]).T
    # cv2.imshow('Depth - original image', 1. - depthD2d / depthD2d.flatten().max())
    
    # Disparities visualizing
    print np.unique(np.array(disparityMap.flatten(), dtype=np.int32))
    nonocclDisparity = disparityMap != 0.0
    flattenedDisparity = disparityMap[nonocclDisparity].flatten()
    maxd, mind = flattenedDisparity.max(), flattenedDisparity.min()
    disparityMapCopy = np.copy(disparityMap)
    disparityMapCopy[nonocclDisparity] = (disparityMapCopy[nonocclDisparity]-mind)*255/(maxd-mind)
    disparityMapCopy = cv2.applyColorMap(np.array(disparityMapCopy, dtype=np.uint8), cv2.COLORMAP_JET)
    cv2.imshow('Disparity', disparityMapCopy)
    
    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    ax.invert_xaxis()       #YOLO
    plt.show()
