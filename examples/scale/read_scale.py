#!/usr/bin/python2.7
"""
Example
1. Set the robot in teaching mode
2. Record the trajectory (in RL the left arm will just drop down, on the real robot the user can move it in
gravity compensation)
3. The arm will be brought to the initial position of the learned movement
4. The movement learnt will be played

Everything will happen in the cartesian space.
"""

from robopy.hardware.scale import DymoScale, ScaleObserver

if __name__ == "__main__":

    scale = DymoScale()
    scale_observer = ScaleObserver(scale)
    print("The current weight measured is: %dg" % scale_observer("DYMO_WEIGHT")["DYMO_WEIGHT"])

