#!/usr/bin/env python

class Airmar:
    def __init__(self):
        self.heading = 0
        self.amrRoll = 0
        self.apWndDir = 0
        self.apWndSpd = 0
        self.cog = 0
        self.lat = 0
        self.long = 0
        self.sog = 0
        self.truWndDir = 0
        self.truWndSpd = 0

if __name__ == '__main__':
    try:
        am = Airmar()
