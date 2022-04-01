#!/usr/bin/python2.7

from robopy.hardware.scale import DymoScale, ScaleObserver

if __name__ == "__main__":

    scale = DymoScale()
    scale_observer = ScaleObserver(scale)
    print("The current weight measured is: %dg" % scale_observer("DYMO_WEIGHT")["DYMO_WEIGHT"])

