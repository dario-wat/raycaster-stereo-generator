import sys
import time
import math

import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rcutils
from geometry import plotT, plotP, plotL, plotG, plotImVs, rayCaster, testCoords, createGridImage, \
    fillDisparityMap, noisify

from geometry_cpp import convert_coordinates_2d, rotate_3d, Vector, Triangle, FakeRect, \
    create_grid, raycast, depth_to_scene, missing_interpolation, transform_virtual

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

# Camera parameters
Z_HEIGHT = 3.
X_OFF = 0.01
Y_OFF = -0.15
B = -0.18
TILT = 0.
CHIP_MUL = 1.0

# Intrinsic parameters
focalLength = 0.00315

# Default vector and camera position that is later used to rotate and place image plane
# easier in 3D space. Includes perpendicular vectors for rotating the side image vectors.
defaultVecX = Vector(1., 0., 0.)
defaultVecY = Vector(0., 1., 0.)
defaultVecDir = Vector(0., 0., -1.)
perpX = defaultVecDir.cross(defaultVecX)
perpY = defaultVecDir.cross(defaultVecY)

# TODO, do something with this function, it uses so many global things
def positionCamera(point, direction, rotAngle):
    """Highly globally dependent function. Very complicated geometry positioning. Must not be
    called before all intrinsic parameters are set."""
    perpendicularAxis = direction.cross(defaultVecDir)
    angle = math.atan2(perpendicularAxis.norm(), direction*defaultVecDir)
    focalVec, vecxx, vecyy = (focVec, imVecX, imVecY) if defaultVecDir.cross(direction) == ZEROVEC \
        else rotate_3d(perpendicularAxis, angle, [focVec, imVecX, imVecY])
    vecxx, vecyy = rotate_3d(focalVec, rotAngle, [vecxx, vecyy])
    vecbx = (focalVec - vecxx).normalize() * widthP
    vecby = (focalVec - vecyy).normalize() * heightP
    origin = point + focalVec - vecbx * (widthPxl-1) / 2. - vecby * (heightPxl-1) / 2.
    return focalVec, vecbx, vecby, origin

if __name__ == '__main__':
    if len(sys.argv) <= 2:
        print 'Need file name'
        exit()

    # Read up the file
    vertices, triangles = rcutils.readFace(sys.argv[1])
    
    # Create plotter and add the scene to it
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    collection = rcutils.createCollection(vertices, triangles)[:len(triangles)-2]
    ax.add_collection3d(Poly3DCollection(collection, facecolors='0.75', edgecolors='k'))

    # Read image and update parameters
    origImg = cv2.imread(sys.argv[2])
    # cv2.imwrite('gridimg.pgm', createGridImage(640, 480, 3)); exit()

    # Intrinsic camera parameters
    widthPxl = origImg.shape[1]
    heightPxl = origImg.shape[0]
    widthChip = 0.0000014 * 2600 * CHIP_MUL
    heightChip = 0.0000014 * 1952 * CHIP_MUL
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
        transform_virtual(int(widthPxl), int(heightPxl), resultCoords, list(map(int, origImg[:,:,0].flatten())))
    virtualImg = np.array(virtualImg, dtype=np.uint8).reshape([heightPxl, widthPxl])
    disparityMap = np.array(disparityMap, dtype=np.float32).reshape([heightPxl, widthPxl])
    virtualVisual = np.array(virtualVisual, dtype=np.uint8).reshape([heightPxl, widthPxl, 3])
    occlusionMask = np.array(occlusionMask, dtype=np.uint8).reshape([heightPxl, widthPxl])
    virtualImgFilled = np.array(missing_interpolation(
        list(map(int, virtualImg.flatten())), 
        virtualImg.shape[1],
        virtualImg.shape[0],
        200,
        list(map(int, occlusionMask.flatten()))), dtype=np.uint8).reshape((heightPxl, widthPxl))
    print 'Time:', time.time() - start
    
    # Visualizing everything
    # cv2.imshow('Right image (real)', cv2.resize(origImg, (640, 480)))
    # cv2.imshow('Left image (virtual)', virtualImg)
    # cv2.imshow('Occlusion mask', occlusionMask)
    # cv2.imshow('Virtual with pink occlusions', cv2.resize(virtualVisual, (640, 480)))
    cv2.imshow('Virtual image non-occluded', virtualImgFilled)

    # depthS2d = np.array(depthS).reshape([widthPxl, heightPxl]).T
    # depthSimg = np.array((1. - depthS2d / depthS2d.flatten().max())*255, dtype=np.uint8)
    # cv2.imshow('Depth - virtual image', depthSimg)

    # foreground = np.array(foreground, dtype=np.uint8).reshape([widthPxl, heightPxl]).T * 255
    # cv2.imshow('Foreground', foreground)
    # cv2.imshow('Intersect face', cv2.resize(virtualImg | foreground, (640, 480)))

    # depth to drain, not useful
    # depthD = depth_to_scene(grid2, drain, triangles_cpp)
    # depthD2d = np.array(depthD).reshape([widthPxl, heightPxl]).T
    # depthDimg = np.array((1. - depthD2d / depthD2d.flatten().max())*255, dtype=np.uint8)
    # cv2.imshow('Depth - original image', depthDimg)
    
    # Disparities visualizing
    nonocclDisparity = disparityMap >= 0.0
    flattenedDisparity = disparityMap[nonocclDisparity].flatten()
    print np.unique(flattenedDisparity)
    maxd, mind = flattenedDisparity.max(), flattenedDisparity.min()
    print 'max, min', maxd, mind

    disparityMap = fillDisparityMap(disparityMap, 6)    
    disparityMapCopy = np.copy(disparityMap)
    disparityMapCopy = (disparityMapCopy-24)*255/14
    disparityMapCopy = cv2.applyColorMap(np.array(disparityMapCopy, dtype=np.uint8), cv2.COLORMAP_JET)
    cv2.imshow('Disparity', disparityMapCopy)

    # Adding noise
    imgNoise = noisify(origImg[:,:,0], 0.5)
    cv2.imwrite('output_data/orig_0_5.pgm', imgNoise)
    imgNoise = noisify(origImg[:,:,0], 1.0)
    cv2.imwrite('output_data/orig_1_0.pgm', imgNoise)
    imgNoise = noisify(origImg[:,:,0], 2.0)
    cv2.imwrite('output_data/orig_2_0.pgm', imgNoise)

    imgNoise = noisify(virtualImgFilled, 0.5)
    cv2.imwrite('output_data/virt_0_5.pgm', imgNoise)
    imgNoise = noisify(virtualImgFilled, 1.0)
    cv2.imwrite('output_data/virt_1_0.pgm', imgNoise)
    imgNoise = noisify(virtualImgFilled, 2.0)
    cv2.imwrite('output_data/virt_2_0.pgm', imgNoise)
    
    # cv2.imwrite('output_data/origimg.pgm', origImg[:,:,0])
    # cv2.imwrite('output_data/origimg_P6.pgm', origImg)
    # cv2.imwrite('output_data/virtualimg.pgm', virtualImg)
    # cv2.imwrite('output_data/virtualimg_pink.pgm', virtualVisual)
    # cv2.imwrite('output_data/virtualimg_nooccl.pgm', virtualImgFilled)
    # cv2.imwrite('output_data/virtualimg_nooccl_P6.pgm', cv2.cvtColor(virtualImgFilled, cv2.COLOR_GRAY2BGR))
    # cv2.imwrite('output_data/occlusion_mask.pgm', occlusionMask)
    # cv2.imwrite('output_data/foreground_mask.pgm', foreground)
    # cv2.imwrite('output_data/depth_source.pgm', depthSimg)
    # cv2.imwrite('output_data/depth_drain.pgm', depthDimg)
    # cv2.imwrite('output_data/disparity.pgm', disparityMapCopy)
    # cv2.imwrite('output_data/virtualimg_fg.pgm', virtualImg | foreground)
    # cv2.imwrite('output_data/disparity_map.pgm', disparityMap)
    # np.save('output_data/disparity_data', disparityMap)


    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    ax.invert_xaxis()       #YOLO

    print 'All done:', time.time() - start

    plt.show()


# Params for both large images in data_large_{1,2}
# # Camera parameters
# Z_HEIGHT = 2.
# X_OFF = -0.09
# Y_OFF = 0.15
# B = -0.18
# TILT = 0.
# CHIP_MUL = 1.0

# # Intrinsic parameters
# focalLength = 0.00375
# min max 123 200


# vga_data_1
# # Camera parameters
# Z_HEIGHT = 3.
# X_OFF = 0.01
# Y_OFF = -0.15
# B = -0.18
# TILT = 0.
# CHIP_MUL = 1.0

# # Intrinsic parameters
# focalLength = 0.00315
# min max diff 24 38 14 for scaling the color and sizes 
# actually 26 and 36

# vga_data_2
# Camera parameters
# Z_HEIGHT = 2.8
# X_OFF = 0.2
# Y_OFF = -0.15
# B = -0.18
# TILT = 0.
# CHIP_MUL = 1.0

# # Intrinsic parameters
# focalLength = 0.00315
# min max 27 38

# vga_data_3
# Camera parameters
# Z_HEIGHT = 3.1
# X_OFF = 0.05
# Y_OFF = 0.3
# B = -0.18
# TILT = 0.
# CHIP_MUL = 1.0

# # Intrinsic parameters
# focalLength = 0.00315
# min max 25 34

# vga_data_4
# Camera parameters
# Z_HEIGHT = 3.1
# X_OFF = 0.2
# Y_OFF = 0.3
# B = -0.18
# TILT = 0.
# CHIP_MUL = 1.0

# # Intrinsic parameters
# focalLength = 0.00315
# min max 25 34

# vga_data_5
# Camera parameters
# Z_HEIGHT = 3.4
# X_OFF = 0.8
# Y_OFF = 0.3
# B = -0.18
# TILT = 0.
# CHIP_MUL = 1.0

# # Intrinsic parameters
# focalLength = 0.00315
# min max 23 31