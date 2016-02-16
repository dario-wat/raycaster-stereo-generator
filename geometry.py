from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from geometry_cpp import intersects_scene, intersect_ray_scene, Ray, intersect_ray_fakerect

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

def rayCaster(grid, source, sceneTriangles, drain, drainRect, veca, vecb, axp):
    """Seems complicated, read raycast and convert_coordinates_2d in cpp."""
    # basis computations
    vecx = veca / veca.norm()
    vecy = vecb / vecb.norm()
    ax, ay, az = vecx
    bx, by, bz = vecy
    
    coords = []
    for gridPoint in grid:
        ray = Ray(source, gridPoint)
        intersFound, intersPoint, triangleIdx = intersect_ray_scene(ray, sceneTriangles)
        if intersFound != 1:  # this should always be false
            coords.append(None)
            continue
        
        # plotL(axp, drain, intersPoint, color='r')
        # plotL(axp, source, intersPoint, color='b')
        # plotP(axp, intersPoint, color='r')

        backRay = Ray(intersPoint, drain)
        intersBackFound, intersBackPoint = intersect_ray_fakerect(backRay, drainRect)
        if intersBackFound != 1 or intersects_scene(backRay, sceneTriangles, triangleIdx):
            coords.append(None)
            continue
        
        # In cpp this is implemented in separate function to separate logic
        # here it is used for debugging purposes
        translatedRectPoint = intersBackPoint - drainRect.origin()
        cx, cy, cz = translatedRectPoint
        m = (cy - by*cx/bx) / (ay - by*ax/bx)
        n = (cy - ay*cx/ax) / (by - ay*bx/ax)
        coords.append((m, n))
        
        # plotL(axp, offset, offset + n*vecy)
        # plotL(axp, offset + n*vecy, offset + m*vecx + n*vecy)
        # plotL(axp, offset, offset + veca)
        # plotL(axp, offset, offset + vecb)
        # print m / veca.norm(), n / vecb.norm()
        # plotP(axp, intersBackPoint)
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