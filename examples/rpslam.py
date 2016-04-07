#!/usr/bin/env python

'''
rpslam.py : BreezySLAM Python with RPLidar and Visual Light Positioning from mercator (http://github.com/kareldecoster/mercator)

Author: Karel De Coster
Github: http://github.com/kareldecoster/BreezySLAM
Date: 2016-4-7

based on xvslam.py by Simon D. Levy (http://github.com/simondlevy/breezyslam)

                
Copyright (C) 2016 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 3
LIDAR_DEVICE            = '/dev/ttyACM0'

from breezyslam.algorithms import VLP_SLAM
from breezyslam.components import XVLidar as LaserModel

from xvlidar import XVLidar as Lidar

from cvslamshow import SlamShow

if __name__ == '__main__':

    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = VLP_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    display = SlamShow(MAP_SIZE_PIXELS, MAP_SIZE_METERS*1000/MAP_SIZE_PIXELS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    
    #wait for positioning to be ready
    ready = "0"
    while ready = "0"
    	file_isup = open("/var/lib/mercator/isup","r");
    	ready = file_isup.read(1);
    	file_isup.close();

    while True:

        # Update SLAM with current Lidar scan, using first element of (scan, quality) pairs
        slam.update([pair[0] for pair in lidar.getScan()])

        # Get current robot position
        x, y, theta = slam.getpos()

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        display.displayMap(mapbytes)

        display.displayRobot((x, y, theta))

        trajectory.append((x,y))

        # Display trajectory
        display.displayTrajectory(trajectory)

        # Exit on ESCape
        key = display.refresh()
        if key != None and (key&0x1A):
            exit(0)
     
