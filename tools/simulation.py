#!/usr/bin/python

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin, radians, degrees
import numpy as np

def _cos(deg):
    return np.cos(np.radians(deg))

def _sin(deg):
    return np.sin(np.radians(deg))

def forward_kinematics(coxa, femur, tibia, theta, alpha, beta):
    print "Solving forward kinematics to verify things look OK"
    print "Coxa: {0} ({1})", coxa, type(coxa)
    print "Femur {0}: ({1})", femur, type(femur)
    print "Tibia: {0} ({1})", tibia, type(tibia)
    print "Theta: {0} ({1})", theta, type(theta)
    print "Alpha: {0} ({1})", alpha, type(alpha)
    print "Beta: {0} ({1})", beta, type(beta)
    # Create the homeogenious transformation matrices
    a1 = np.matrix([[_cos(theta),   -1 * _sin(theta) * _cos(-90),   _sin(theta) * _sin(-90),     coxa * _cos(theta)],
                    [_sin(theta),   _cos(theta) * _cos(-90),        -1 * _cos(theta) * sin(-90), coxa * _sin(theta)],
                    [0,             _sin(-90),                      _cos(-90),                  0],
                    [0,             0,                              0,                          1]])

    a2 = np.matrix([[_cos(alpha),   -1 * _sin(alpha) * _cos(0),   _sin(alpha) * _sin(0),     femur * _cos(alpha)],
                    [_sin(alpha),   _cos(alpha) * _cos(0),        -1 * _cos(alpha) * sin(0), femur * _sin(alpha)],
                    [0,             _sin(0),                      _cos(0),                   0],
                    [0,             0,                              0,                      1]])

    a3 = np.matrix([[_cos(beta),   -1 * _sin(beta) * _cos(0),   _sin(beta) * _sin(0),     tibia * _cos(beta)],
                    [_sin(beta),   _cos(beta) * _cos(0),        -1 * _cos(beta) * sin(0), tibia * _sin(beta)],
                    [0,             _sin(0),                      _cos(0),                   0],
                    [0,             0,                              0,                      1]])

    point = np.matrix([[0],[0],[0],[1]])
    point1 = a1 * point
    point2 = a1 * a2 * point
    point3 = a1 * a2 * a3 * point
    x = []
    y = []
    z = []
    x.append(point.item(0,0))
    x.append(point1.item(0,0))
    x.append(point2.item(0,0))
    x.append(point3.item(0,0))

    y.append(point.item(1,0))
    y.append(point1.item(1,0))
    y.append(point2.item(1,0))
    y.append(point3.item(1,0))

    z.append(point.item(2,0))
    z.append(point1.item(2,0))
    z.append(point2.item(2,0))
    z.append(point3.item(2,0))

    ax1.clear()
    ax1.plot_wireframe(x, y, z)




def animate(i):
    data = np.loadtxt("../src/foobar")
    forward_kinematics(*data)

fig = plt.figure()
ax1 = fig.add_subplot(111, projection='3d')
ax1.set_xlim(0, 100)
ax1.set_ylim(0, 100)

ax1.set_xlabel('x axis')
ax1.set_ylabel('y axis')
ax1.set_zlabel('z axis')
ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()

