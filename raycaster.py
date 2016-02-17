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
    create_grid, raycast

# focal length: 5.15 mm
# sensor width: 2600 pixels
# sensor height: 1952 pixels
# pixel width: 0.0014 mm
# pixel height: 0.0014 mm
# subsampled height: 488 px
# subsampled width: 656 px
# focal length in px = 5.15/(0.0014*0.5*(2600/656+1952/488)) = 924

ZEROVEC = Vector(0., 0., 0.)

# Intrinsic camera parameters
widthPxl = 384
heightPxl = 288
widthChip = 0.8
heightChip = 0.6
focalLength = 1.5
widthC = widthChip * (widthPxl-1) / widthPxl
heightC = heightChip * (heightPxl-1) / heightPxl
widthP = widthChip / widthPxl
heightP = heightChip / heightPxl

# Intrinsic calculations that do not depend on camera position
# Default vector and camera position that is later used to rotate and place image plane
# easier in 3D space. Includes perpendicular vectors for rotating the side image vectors.
defaultVecX = Vector(1., 0., 0.)
defaultVecY = Vector(0., 1., 0.)
defaultVecDir = Vector(0., 0., -1.)
perpX = defaultVecDir.cross(defaultVecX)
perpY = defaultVecDir.cross(defaultVecY)

# Angle between the focal vector and vector that touches the half point of the side
# of the image. Length of the said vector and the vector itself.
angleX = math.atan(widthC/2./focalLength)
angleY = math.atan(heightC/2./focalLength)
lenX = math.sqrt(focalLength*focalLength + widthC*widthC/4.)
lenY = math.sqrt(focalLength*focalLength + heightC*heightC/4.)
imVecX = rotate_3d(perpX, angleX, [defaultVecDir])[0].normalize() * lenX
imVecY = rotate_3d(perpY, angleY, [defaultVecDir])[0].normalize() * lenY
focVec = defaultVecDir * focalLength

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

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print 'Need file name'
        exit()

    # Read up the file
    vertices, triangles = rcutils.readFace(sys.argv[1])
    
    # Create plotter and add the scene to it
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    collection = rcutils.createCollection(vertices, triangles)
    ax.add_collection3d(Poly3DCollection(collection, facecolors='0.75', edgecolors='k'))

    # Source camera
    rotAngleS = 0
    source = Vector(0., 0., 3.)
    cameraDirS = Vector(0., 0., -1.).normalize()
    focVecS, vecbxs, vecbys, originS = positionCamera(source, cameraDirS, rotAngleS)
    # plotImVs(ax, source, focVecS, vecbxs, vecbys, widthPxl, heightPxl)
    
    grid = create_grid(widthPxl, heightPxl, vecbxs, vecbys, originS)
    plotG(ax, grid, 'r')

    # Drain camera
    rotAngleD = 0
    drain = Vector(.5, 0., 3.)
    cameraDirD = Vector(0., 0., -1.).normalize()
    focVecD, vecbxd, vecbyd, originD = positionCamera(drain, cameraDirD, rotAngleD)
    # plotImVs(ax, drain, focVecD, vecbxd, vecbyd, widthPxl, heightPxl)
    
    # Drain grid plotting
    grid2 = create_grid(widthPxl, heightPxl, vecbxd, vecbyd, originD)
    # plotG(ax, grid2, 'b')
    
    # drain grid rectangle
    drainRectangle = FakeRect(originD, vecbxd*(widthPxl-1), vecbyd*(heightPxl-1))
    plotT(ax, drainRectangle.t1)
    plotT(ax, drainRectangle.t2)

    # Create triangles to be used for intersections
    triangles_cpp = []
    for t in triangles:
        triangles_cpp.append(
            Triangle(Vector(*vertices[t[0]]), Vector(*vertices[t[1]]), Vector(*vertices[t[2]])))

    # The actual raycasting
    start = time.time()
    # coords = rayCaster(grid, source, triangles_cpp, drain, drainRectangle, ax)
    coords2 = raycast(grid, source, triangles_cpp, drain, drainRectangle)
    print time.time() - start

    res = convert_coordinates_2d(coords2, originD, vecbxd, vecbyd, rotAngleD)
    # print time.time() - start
    # for p in filter(lambda g: g is not None, res):
    #     print p


    origImPxls = np.mgrid[0:widthPxl, 0:heightPxl].reshape(2, widthPxl*heightPxl).T
    virtualImPxl = np.array(res)

    img = cv2.imread(sys.argv[2])
    cv2.imshow('Nice one', img)
    img2 = np.zeros((heightPxl, widthPxl), np.uint8)
    
    for i in xrange(len(virtualImPxl)):
        if virtualImPxl[i] is None:
            continue
        j, k = virtualImPxl[i]
        y, x = origImPxls[i]
        img2[k,j] = img[y,x,0]
    cv2.imshow('Nicer one', img2)

    # print virtualImPxl.shape, origImPxls.shape

    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    plt.show()
