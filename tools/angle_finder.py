#!/usr/bin/python


import numpy as np
import sys


def _cos(deg):
    return np.cos(np.radians(deg))

def _sin(deg):
    return np.sin(np.radians(deg))

def solve(x, y, deg):
    mult = np.matrix([[_cos(deg), -1*_sin(deg)],
                      [_sin(deg), _cos(deg)]])
    pt = np.matrix([[x],[y]])
    new_pt = mult * pt
    print "{{{0}, {1}, DOWN}},".format(
    int(round(new_pt.item(0,0))),
    int(round(new_pt.item(1,0)))),

def main():
    print "Calculating angles"
    try:
        _file = open("./angle_of_the_dangle.csv")
    except:
        print "Failed to open file"
        sys.exit(-1)
    for line in _file:
        angles = [float(x) for x in line.split(",")]
        solve(-120, 120, angles[0])
        solve(120, 120, angles[1])
        solve(-120, -120, angles[2])
        solve(120, -120, angles[3])
        print ""


if __name__ == "__main__":
    main()
