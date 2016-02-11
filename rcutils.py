import numpy as np

""" Has all the util functions that are not directly related to raycasting or any
kind of geometry. """

def readFace(filename):
    """Takes a filename of a face file (works with candide) and read vertices and face triangles.
        Returns a numpy array of vertices and numpy array of triangle indices. """
    with open(filename) as faceFile:
        lines = map(lambda l: l.rstrip(), faceFile.readlines())
        lines = filter(lambda l: l and l[0] != '#', lines)
        
        n = int(lines[0])
        vertexList = np.array(map(lambda s: s.split(), lines[1:1+n]), dtype=np.float64)

        lines = lines[1+n:]
        m = int(lines[0])
        faceList = np.array(map(lambda s: s.split(), lines[1:1+m]), dtype=np.int64)
        return vertexList, faceList

def createCollection(vertices, triangles):
    """Creates a collection of triangle vertices that will be used
        as a polygon collection for drawing the face."""
    return map(lambda idx: vertices[idx], triangles)
