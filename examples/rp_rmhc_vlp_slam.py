#!/usr/bin/env python

'''
rpslam.py : BreezySLAM Python with RPLidar and Visual Light Positioning from mercator (http://github.com/kareldecoster/mercator)

based on xcslam.py by Simon D. Levy (http://github.com/simondlevy/breezyslam)

                
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
LIDAR_DEVICE            = '/dev/ttyO1'

from breezyslam.algorithms import VLP_RMHC_SLAM
from breezyslam.components import RPLidar as LaserModel

from rplidar import RPLidar

from PIL import Image

import sys
import signal
import time

#GLOBALS
done = 0

def mm2pix(mm):
        
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
    
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        #sys.exit(0)
        global done 
        done = 1
        
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    # Connect to Lidar unit
    lidar = RPLidar(LIDAR_DEVICE)
    
    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = VLP_RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    print "starting sleep.."
    time.sleep(120)
    print "waking up.."
    while done == 0:

        # Update SLAM with current Lidar scan, using first element of (scan, quality) pairs
        slam.update([pair[0] for pair in lidar.getScan()])
        distances = [pair[0] for pair in lidar.getScan()]
        my_angle = 0
        print "NEW MEASUREMENT"
        while my_angle < 360:
            print "          angle = {angle}       distance = {distance}".format(angle = my_angle, distance = distances[my_angle])
            my_angle = my_angle + 1
        # Get current robot position
        x, y, theta = slam.getpos()
        
        trajectory.append((x,y))
        
    # Get current map bytes as grayscale
    slam.getmap(mapbytes)

    # Put trajectory into map as black pixels
    for coords in trajectory:
                
        x_mm, y_mm = coords
                               
        x_pix = mm2pix(x_mm)
        y_pix = mm2pix(y_mm)
                                                                                              
        mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0;
                    
    # Save map and trajectory as PNG file
    image = Image.frombuffer('L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), mapbytes, 'raw', 'L', 0, 1)
    image.save('slam_map.png')
    
    # Tell lidar to shutdown
    lidar.set_exitflag()

    exit(0)
     
