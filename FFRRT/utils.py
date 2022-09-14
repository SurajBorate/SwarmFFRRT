import math
from typing import List
import numpy as np
import cv2

from rrt2d import Node

def save(img, fname):
    cv2.imwrite(f'imgs/{fname}.png', img)

class Vector(object):
    def __init__(self, *args):
        """ Create a vector, example: v = Vector(1,2) """
        if len(args) == 0:
            self.values = (0, 0)
        else:
            self.values = args

    def norm(self):
        """ Returns the norm (length, magnitude) of the vector """
        return math.sqrt(sum(x*x for x in self))

    def argument(self, radians=False):
        """ Returns the argument of the vector, the angle clockwise from +y. In degress by default, 
            set radians=True to get the result in radians. This only works for 2D vectors. """
        arg_in_rad = math.acos(Vector(0, 1)*self/self.norm())
        if radians:
            return arg_in_rad
        arg_in_deg = math.degrees(arg_in_rad)
        if self.values[0] < 0:
            return 360 - arg_in_deg
        else:
            return arg_in_deg

    def normalize(self):
        """ Returns a normalized unit vector """
        norm = self.norm()
        if norm == 0:
            return Vector(0, 0)
        normed = tuple(x / norm for x in self)
        return self.__class__(*normed)

    def rotate(self, theta):
        """ Rotate this vector. If passed a number, assumes this is a 
            2D vector and rotates by the passed value in degrees.  Otherwise,
            assumes the passed value is a list acting as a matrix which rotates the vector.
        """
        if isinstance(theta, (int, float)):
            # So, if rotate is passed an int or a float...
            if len(self) != 2:
                raise ValueError(
                    "Rotation axis not defined for greater than 2D vector")
            return self._rotate2D(theta)

        matrix = theta
        if not all(len(row) == len(self) for row in matrix) or not len(matrix) == len(self):
            raise ValueError(
                "Rotation matrix must be square and same dimensions as vector")
        return self.matrix_mult(matrix)

    def _rotate2D(self, theta):
        """ Rotate this vector by theta in degrees.

            Returns a new vector.
        """
        theta = math.radians(theta)
        # Just applying the 2D rotation matrix
        dc, ds = math.cos(theta), math.sin(theta)
        x, y = self.values
        x, y = dc*x - ds*y, ds*x + dc*y
        return self.__class__(x, y)

    def matrix_mult(self, matrix):
        """ Multiply this vector by a matrix.  Assuming matrix is a list of lists.

            Example:
            mat = [[1,2,3],[-1,0,1],[3,4,5]]
            Vector(1,2,3).matrix_mult(mat) ->  (14, 2, 26)

        """
        if not all(len(row) == len(self) for row in matrix):
            raise ValueError('Matrix must match vector dimensions')

        # Grab a row from the matrix, make it a Vector, take the dot product,
        # and store it as the first component
        product = tuple(Vector(*row)*self for row in matrix)

        return self.__class__(*product)

    def inner(self, vector):
        """ Returns the dot product (inner product) of self and another vector
        """
        if not isinstance(vector, Vector):
            raise ValueError('The dot product requires another vector')
        return sum(a * b for a, b in zip(self, vector))

    def __mul__(self, other):
        """ Returns the dot product of self and other if multiplied
            by another Vector.  If multiplied by an int or float,
            multiplies each component by other.
        """
        if isinstance(other, Vector):
            return self.inner(other)
        elif isinstance(other, (int, float)):
            product = tuple(a * other for a in self)
            return self.__class__(*product)
        else:
            raise ValueError(
                "Multiplication with type {} not supported".format(type(other)))

    def __rmul__(self, other):
        """ Called if 4 * self for instance """
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, Vector):
            divided = tuple(self[i] / other[i] for i in range(len(self)))
        elif isinstance(other, (int, float)):
            divided = tuple(a / other for a in self)
        else:
            raise ValueError(
                "Division with type {} not supported".format(type(other)))

        return self.__class__(*divided)

    def __add__(self, other):
        """ Returns the vector addition of self and other """
        if isinstance(other, Vector):
            added = tuple(a + b for a, b in zip(self, other))
        elif isinstance(other, (int, float)):
            added = tuple(a + other for a in self)
        else:
            raise ValueError(
                "Addition with type {} not supported".format(type(other)))

        return self.__class__(*added)

    def __radd__(self, other):
        """ Called if 4 + self for instance """
        return self.__add__(other)

    def __sub__(self, other):
        """ Returns the vector difference of self and other """
        if isinstance(other, Vector):
            subbed = tuple(a - b for a, b in zip(self, other))
        elif isinstance(other, (int, float)):
            subbed = tuple(a - other for a in self)
        else:
            raise ValueError(
                "Subtraction with type {} not supported".format(type(other)))

        return self.__class__(*subbed)

    def __rsub__(self, other):
        """ Called if 4 - self for instance """
        return self.__sub__(other)

    def __iter__(self):
        return self.values.__iter__()

    def __len__(self):
        return len(self.values)

    def __getitem__(self, key):
        return self.values[key]

    def __repr__(self):
        return str(self.values)


class RopeNode():
    def __init__(self, x, y, coeffs=[1, 1], state=None, oppositeNode=None):
        self.x = x

        self.y = y
        self.oppositeNode = oppositeNode
        self.connections = set()
        self.vector = Vector(x, y)
        self.coeffs = coeffs
        self.state = state
        self.fallen = False
        self.disp = 0

    def connect(self, node, terminate=False):

        self.connections.add(node)
        if not terminate:
            node.connect(self, terminate=True)

    def disconnect(self, node):
        self.connections.remove(node)
        node.connections.remove(self)

    def computeGradientForce(self, image):
        y = int(self.y)
        x = int(self.x)
        pel = image[y, x]
        grads = np.array([
            image[y, x+1] - pel,
            image[y+1, x+1] - pel,
            image[y+1, x] - pel,
            image[y+1, x-1] - pel,
            image[y, x-1] - pel,
            image[y-1, x-1] - pel,
            image[y-1, x] - pel,
            image[y-1, x + 1] - pel,
        ])

        direction = np.argmin(grads)

        vectors = [
            Vector(1, 0).normalize(),
            Vector(1, 1).normalize(),
            Vector(0, 1).normalize(),
            Vector(-1, 1).normalize(),
            Vector(-1, 0).normalize(),
            Vector(-1, -1).normalize(),
            Vector(0, -1).normalize(),
            Vector(1, -1).normalize(),
        ]
        return vectors[direction] * float(pel)

    def computeInternalForce(self):
        repulsiveForce = Vector(0, 0)
        for n in self.connections:
            disp = n.vector - self.vector

            norm = disp.norm()
            thresh = [2, 5]
            if thresh[0] < norm < thresh[1]:
                mult = 0
            elif norm < thresh[0]:
                mult = -1
            else:
                mult = 1

            repulsiveForce += mult * (disp)
        return repulsiveForce

    def constrainForce(self, force, image):
        y = int(self.y)
        x = int(self.x)
        pel = image[y, x]
        grads = np.array([
            image[y, x+1] - pel,
            image[y+1, x+1] - pel,
            image[y+1, x] - pel,
            image[y+1, x-1] - pel,
            image[y, x-1] - pel,
            image[y-1, x-1] - pel,
            image[y-1, x] - pel,
            image[y-1, x + 1] - pel,
        ])
        grads = np.abs(grads)
        vectors = [
            Vector(1, 0).normalize(),
            Vector(1, 1).normalize(),
            Vector(0, 1).normalize(),
            Vector(-1, 1).normalize(),
            Vector(-1, 0).normalize(),
            Vector(-1, -1).normalize(),
            Vector(0, -1).normalize(),
            Vector(1, -1).normalize(),
        ]
        direction = np.argmin(grads)

        return (vectors[direction] * force) * vectors[direction]

    def computeForce(self, image):
        externalForce = self.computeGradientForce(image)
        internalForce = self.computeInternalForce()
        externalcoeff = self.coeffs[0]

        return externalForce * externalcoeff + internalForce * self.coeffs[1]

    def step(self, image, t, offsetForce=Vector(0, 0)):
        force = self.computeForce(image)

        force += offsetForce * 0.1
        if self.disp <= 50:

            self.y += force[1] * dt
            self.x += force[0] * dt

            self.vector = Vector(self.x, self.y)
            self.disp += np.hypot(force[0], force[1]) * dt

        cv2.circle(t, (int(self.x), int(self.y)), 3, (0, 0, 255), -1)
        cv2.line(t, (int(self.x), int(self.y)), (int(
            self.x + force[0] * 2), int(self.y + force[1] * 2)), (0, 128, 0), 1)
        for n in self.connections:
            cv2.line(t, (int(self.x), int(self.y)),
                     (int(n.x), int(n.y)), (128, 128, 128), 1)

        return t

    def __str__(self):
        return "({}, {})".format(self.x, self.y)


dt = 0.01


class Rope:
    def __init__(self, nodes: List[RopeNode]):

        self.nodes = nodes

        hwidth = len(nodes)/2.
        scalea = 1
        scaleb = 5
        for i in range(len(nodes)):

            # ca = (abs(hwidth - i)/hwidth)  * scalea + 1
            # cb = (1 - abs(hwidth - i)/hwidth)  * scaleb + 3
            # self.nodes[i].coeffs = [ca, cb]
            self.nodes[i].coeffs = [1, 2]
        for i in range(len(nodes)//3, len(nodes)//3 * 2):
            # ca = (abs(hwidth - i)/hwidth)  * scalea + 1
            self.nodes[i].coeffs = [1, 1]

        for i in range(-4, -1):
            self.nodes[i].coeffs = [10, 3]

        self.nodes[0].coeffs = [0, 0]
        self.nodes[-1].coeffs = [0, 0]
        self.disp = Vector(0, 0)

    def step(self, gradimg, vizimg):

        for node in self.nodes:
            vizimg = node.step(gradimg, vizimg)

        return vizimg

    def isConverged(self, gradImg, vizimg, viz = True, distLen = 2):
        distances = []
        k = 0
        
        while True:
            self.disp = 0

            for i in range(15):
                t = vizimg.copy()
                t = self.step(gradImg, t)
                for node in self.nodes:
                    self.disp += (node.computeForce(gradImg) * dt).norm()

            # if i % == 0:
            if k == 0:
                save(t, 'it1')
                k += 1
            if viz:
                show(t, waitkey=1)

            distances.append(self.disp)
            threshdist = abs(np.average(distances[::-1][:8]) - distances[-1])
            if len(distances) > distLen and (threshdist) < 5:
                break

        return t


def convertRopeToTree(rope):
    i = 0
    nodes = []
    for i in range(len(rope.nodes)-1, -1, -1):
        nodes.append(
            Node([
                rope.nodes[i].vector[0], 
                rope.nodes[i].vector[1], 
                rope.nodes[i].state[2],
                rope.nodes[i].state[3],
                rope.nodes[i].state[4],
                ]))

    for i in range(1, len(nodes)):
        nodes[i].parent = nodes[i-1]
    
    for i in range(1, len(nodes) - 1):
        nodes[i].state[2:] = (nodes[i+ 1].state[2:] + nodes[i-1].state[2:] + nodes[i].state[2:]) / 3
    
    return nodes


def convertTreeToRope(node, slack=2):

    rope = []

    while node is not None:
        for i in range(slack):
            rope.append(
                RopeNode(node.state[0], node.state[1], state=node.state))
        node = node.parent

    if slack != 1:
        for i in range(slack):
            rope.pop(1)
            rope.pop(-2)

    for i in range(1, len(rope)):
        rope[i].connect(rope[i-1])
    rope = Rope(rope)
    return rope


def show(img, winname='a', waitkey=0):
    temp = img.copy().astype(float)
    temp /= temp.max()
    cv2.namedWindow(winname, cv2.WINDOW_NORMAL)
    cv2.imshow(winname, temp)

    if cv2.waitKey(waitkey) == ord('q'):
        exit()
