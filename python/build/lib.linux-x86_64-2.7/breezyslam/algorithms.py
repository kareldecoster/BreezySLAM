'''
BreezySLAM: Simple, efficient SLAM in Python

VLP classes added by 
Author: Karel De Coster
Github: http://github.com/kareldecoster/BreezySLAM
Date: 2016-4-7

algorithms.py: SLAM algorithms

Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

# distanceScanToMap is implemented as a C extension for efficiency
from pybreezyslam import distanceScanToMap

import pybreezyslam

import math
import time

# Basic params
_DEFAULT_MAP_QUALITY         = 50 # out of 255
_DEFAULT_HOLE_WIDTH_MM       = 600

# Random mutation hill-climbing (RMHC) params
_DEFAULT_SIGMA_XY_MM         = 100
_VLP_SIGMA_XY_MM			 = 100
_DEFAULT_SIGMA_THETA_DEGREES = 20
_VLP_SIGMA_THETA_DEGREES     = 30
_DEFAULT_MAX_SEARCH_ITER     = 1000
_VLP_MAX_SEARCH_ITER	     = 10000


# CoreSLAM class ------------------------------------------------------------------------------------------------------

class CoreSLAM(object):
    '''
    CoreSLAM is an abstract class that uses the classes Position, Map, Scan, and Laser
    to run variants of the simple CoreSLAM (tinySLAM) algorithm described in 
        
     @inproceedings{coreslam-2010,  
       author    = {Bruno Steux and Oussama El Hamzaoui}, 
       title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},  
       booktitle = {11th International Conference on Control, Automation,   
                    Robotics and Vision, ICARCV 2010, Singapore, 7-10  
                    December 2010, Proceedings},  
       pages     = {1975-1979},  
       publisher = {IEEE},  
       year      = {2010}
     }
    
    
    Implementing classes should provide the method
    
      _updateMapAndPointcloud(scan_mm, velocities)
    
    to update the map and point-cloud (particle cloud).
    '''
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
        map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM):
        '''
        Creates a CoreSLAM object suitable for updating with new Lidar and odometry data.
        laser is a Laser object representing the specifications of your Lidar unit
        map_size_pixels is the size of the square map in pixels
        map_size_meters is the size of the square map in meters
        quality from 0 through 255 determines integration speed of scan into map
        hole_width_mm determines width of obstacles (walls)
        '''
    
        # Initialize parameters
        self.map_quality = map_quality
        self.hole_width_mm = hole_width_mm   
        
        # Store laser for later
        self.laser = laser
        
        # Initialize velocities (dxyMillimeters, dthetaDegrees, dtSeconds) for odometry
        self.velocities = (0, 0, 0)
        
        # Initialize a scan for computing distance to map, and one for updating map
        self.scan_for_distance = pybreezyslam.Scan(laser, 1)
        self.scan_for_mapbuild = pybreezyslam.Scan(laser, 3)
                
        # Initialize the map 
        self.map = pybreezyslam.Map(map_size_pixels, map_size_meters)
                
    def update(self, scans_mm, velocities):
        '''
        Updates the scan and odometry, and calls the the implementing class's _updateMapAndPointcloud method with
        the specified velocities.
         
        scan_mm is a list of Lidar scan values, whose count is specified in the scan_size 
        attribute of the Laser object passed to the CoreSlam constructor
        velocities is a tuple of velocities (dxy_mm, dtheta_degrees, dt_seconds) for odometry
        '''

        # Build a scan for computing distance to map, and one for updating map 
        self._scan_update(self.scan_for_mapbuild, scans_mm)
        self._scan_update(self.scan_for_distance, scans_mm)

        # Update velocities
        velocity_factor = (1 / velocities[2])  if (velocities[2] > 0) else 0
        new_dxy_mm = velocities[0] * velocity_factor  
        new_dtheta_degrees = velocities[1] * velocity_factor
        self.velocities = (new_dxy_mm, new_dtheta_degrees, 0)
                                                  
        # Implementing class updates map and pointcloud
        self._updateMapAndPointcloud(velocities)
        
    def getmap(self, mapbytes):
        '''
        Fills bytearray mapbytes with map pixels, where bytearray length is square of map size passed
        to CoreSLAM.__init__().
        '''
        self.map.get(mapbytes)
        
        
    def __str__(self):
        
        return 'CoreSLAM: %s \n          map quality = %d / 255 \n          hole width = %7.0f mm' % \
               (str(self.map), self.map_quality, self.hole_width_mm)
                
    def __repr__(self):
        
         return self.__str__()

        
    def _scan_update(self, scan, lidar):
        
        scan.update(scans_mm=lidar, hole_width_mm=self.hole_width_mm, velocities=self.velocities)
        
        
# SinglePositionSLAM class ---------------------------------------------------------------------------------------------

class SinglePositionSLAM(CoreSLAM):
    '''
    SinglePositionSLAM is an abstract class that implements CoreSLAM using a point-cloud
    with a single point (position). Implementing classes should provide the method
    
      _getNewPosition(self, start_position)
       
    to compute a new position based on searching from a starting position.
    '''
    
    #This function can be overridden when using VLP to obtain a different starting position
    def _get_start_pos(self, map_size_meters):
        init_coord_mm = 500 * map_size_meters # center of map
        return pybreezyslam.Position(init_coord_mm, init_coord_mm, 0)
        
    def __init__(self, laser, map_size_pixels, map_size_meters, 
                map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM):

        CoreSLAM.__init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality, hole_width_mm)                    
                    
        # Initialize the position (x, y, theta)

        self.position =  self._get_start_pos(map_size_meters)
        
    def _updateMapAndPointcloud(self, velocities):
        '''
        Updates the map and point-cloud (particle cloud). Called automatically by CoreSLAM.update()
        velocities is a tuple of the form (dxy_mm, dtheta_degrees, dt_seconds).
        '''
    
        # Start at current position 
        start_pos = self.position.copy()
        
        # Add effect of velocities
        start_pos.x_mm      += velocities[0] * self._costheta()
        start_pos.y_mm      += velocities[0] * self._sintheta()
        start_pos.theta_degrees = start_pos.theta_degrees +  velocities[1]

        # Add offset from laser
        start_pos.x_mm  += self.laser.offset_mm * self._costheta()
        start_pos.y_mm  += self.laser.offset_mm * self._sintheta()

        # Get new position from implementing class
        new_position = self._getNewPosition(start_pos)
                
        # Update the map with this new position
        self.map.update(self.scan_for_mapbuild, new_position, self.map_quality, self.hole_width_mm)
      
        # Update the current position with this new position, adjusted by laser offset
        self.position = new_position.copy()        
        self.position.x_mm -= self.laser.offset_mm * self._costheta()
        self.position.y_mm -= self.laser.offset_mm * self._sintheta()
  
    def getpos(self):
        '''
        Returns current position as a tuple (x_mm, y_mm, theta_degrees)
        '''
        return (self.position.x_mm, self.position.y_mm, self.position.theta_degrees)
                
        
    def _costheta(self):
        
        return math.cos(self._thetaradians())
 
    def _sintheta(self):
        
        return math.sin(self._thetaradians())
        
    def _thetaradians(self):
        
        return math.radians(self.position.theta_degrees)
        
# RMHC_SLAM class ------------------------------------------------------------------------------------------------------

class RMHC_SLAM(SinglePositionSLAM):
    '''
    RMHC_SLAM implements the _getNewPosition() method of SinglePositionSLAM using Random-Mutation Hill-Climbing
    search.  Uses its own internal pseudorandom-number generator for efficiency.
    '''
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
                map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM,
                random_seed=None, sigma_xy_mm=_DEFAULT_SIGMA_XY_MM, sigma_theta_degrees=_DEFAULT_SIGMA_THETA_DEGREES, 
                max_search_iter=_DEFAULT_MAX_SEARCH_ITER):
        '''
        Creates a RMHCSlam object suitable for updating with new Lidar and odometry data.
        laser is a Laser object representing the specifications of your Lidar unit
        map_size_pixels is the size of the square map in pixels
        map_size_meters is the size of the square map in meters
        quality from 0 through 255 determines integration speed of scan into map
        hole_width_mm determines width of obstacles (walls)
        random_seed supports reproducible results; defaults to system time if unspecified
        sigma_xy_mm specifies the standard deviation in millimeters of the normal distribution of 
           the (X,Y) component of position for RMHC search
        sigma_theta_degrees specifies the standard deviation in degrees of the normal distribution of 
           the rotational component of position for RMHC search
        max_search_iter specifies the maximum number of iterations for RMHC search
        '''
    
        SinglePositionSLAM.__init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality, hole_width_mm)
            
        if not random_seed:
            random_seed = int(time.time()) & 0xFFFF
            
        self.randomizer = pybreezyslam.Randomizer(random_seed)
        
        self.sigma_xy_mm = sigma_xy_mm
        self.sigma_theta_degrees = sigma_theta_degrees
        self.max_search_iter = max_search_iter
        
    def update(self, scan_mm, velocities=None):

        if not velocities:
        
            velocities = (0, 0, 0)
    
        CoreSLAM.update(self, scan_mm, velocities)    
    
    def _getNewPosition(self, start_position):
        '''
        Implements the _getNewPosition() method of SinglePositionSLAM. Uses Random-Mutation Hill-Climbing
        search to look for a better position based on a starting position.
        '''     
        
        # RMHC search is implemented as a C extension for efficiency
        return pybreezyslam.rmhcPositionSearch(
            start_position, 
            self.map, 
            self.scan_for_distance, 
            self.laser,
            self.sigma_xy_mm,
            self.sigma_theta_degrees,
            self.max_search_iter,
            self.randomizer)
                             
    def _random_normal(self, mu, sigma):
        
        return mu + self.randomizer.rnor() * sigma

 # Deterministic_SLAM class  ------------------------------------------------------------------------------------        

class Deterministic_SLAM(SinglePositionSLAM):
    '''
    Deterministic_SLAM implements the _getNewPosition() method of SinglePositionSLAM by simply
    copying the search-start position.
    '''
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
                map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM):
        '''
        Creates a Deterministic_Slam object suitable for updating with new Lidar and odometry data.
        laser is a Laser object representing the specifications of your Lidar unit
        map_size_pixels is the size of the square map in pixels
        map_size_meters is the size of the square map in meters
        quality from 0 through 255 determines integration speed of scan into map
        hole_width_mm determines width of obstacles (walls)
        '''
    
        SinglePositionSLAM.__init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality, hole_width_mm)                    
       
    def _getNewPosition(self, start_position):
        '''
        Implements the _getNewPosition() method of SinglePositionSLAM. Returns a copy of the starting position.
        '''
        
        return start_position.copy()
        
  # VLP_SLAM class ---------------------------------------------------------------------------------------------
  
class VLP_SLAM(SinglePositionSLAM):
    '''
    VLP_SLAM implements the _getNewPosition() method of SinglePositionSLAM by reading in a x and y value
    from a file. The values are expressed in meters. They are written by the mercatord daemon wich uses
    visual light positioning. Mercatord can be found at http://github.com/kareldecoster/mercator
    '''
    
    def __init__(self, laser, map_size_pixels, map_size_meters,
                map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM):
        '''
        Creates a VLP_Slam object suitable for updating with new Lidar and odometry data.
        laser is a Laser object representing the specifications of your Lidar unit
        map_size_pixels is the size of the square map in pixels
        map_size_meters is the size of the square map in meters
        quality from 0 through 255 determines integration speed of scan into map
        hole_width_mm determines width of obstacles (walls)
        '''
        
        SinglePositionSLAM.__init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality, hole_width_mm)    
                            
    def _get_start_pos(self, map_size_meters):
        file_x = open("/var/lib/mercator/x","r")
        x = 1000 * float(file_x.read(10))
        file_x.close()
        file_y = open("/var/lib/mercator/y","r")
        y = 1000 * float(file_y.read(10))
        file_y.close()
        return pybreezyslam.Position(x, y, 0)
           
    def _getNewPosition(self, start_position):
        '''
        Implements the _getNewPosition() method of SinglePositionSLAM. Reads in x and y coordinates from
        files /var/lib/mercator/x and /var/lib/mercator/y. The mercatord daemon writes the position 
        obtained by visual light positioning here. The values posted are in meters. Mercatord can be found at http://github.com/kareldecoster/mercator
        '''
        file_up = open("/var/lib/mercator/isup","r")
        if(int(file_up.read(1)) == 1):
            file_x = open("/var/lib/mercator/x","r")
            self.position.x_mm = 1000*float(file_x.read(10))
            file_y = open("/var/lib/mercator/y", "r")
            self.position.y_mm = 1000*float(file_y.read(10))
            file_theta = open("/var/lib/mercator/theta","r")
            self.position.theta_degrees = float(file_theta.read(10))
            file_x.close()
            file_y.close()
            file_theta.close()

            file_up.close()
            return (self.position.x_mm, self.position.y_mm, self.position.theta_degrees)

        else:
            return start_position.copy()
			
			
# VLP_RMHC_SLAM class ------------------------------------------------------------------------------------------------------

class VLP_RMHC_SLAM(SinglePositionSLAM):
    '''
    VLPRMHC_SLAM implements the _getNewPosition() method of SinglePositionSLAM using Random-Mutation Hill-Climbing
    search.  Uses its own internal pseudorandom-number generator for efficiency. Uses positions acquired from VLP as
    as starting values for the algorithm.
    '''
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=100,
            random_seed=None, sigma_xy_mm=_VLP_SIGMA_XY_MM, sigma_theta_degrees=_VLP_SIGMA_THETA_DEGREES, 
            max_search_iter=_VLP_MAX_SEARCH_ITER):
        '''
        Creates a RMHCSlam object suitable for updating with new Lidar and odometry data.
        laser is a Laser object representing the specifications of your Lidar unit
        map_size_pixels is the size of the square map in pixels
        map_size_meters is the size of the square map in meters
        quality from 0 through 255 determines integration speed of scan into map
        hole_width_mm determines width of obstacles (walls)
        random_seed supports reproducible results; defaults to system time if unspecified
        sigma_xy_mm specifies the standard deviation in millimeters of the normal distribution of 
           the (X,Y) component of position for RMHC search
        sigma_theta_degrees specifies the standard deviation in degrees of the normal distribution of 
           the rotational component of position for RMHC search
        max_search_iter specifies the maximum number of iterations for RMHC search
        '''
    
        SinglePositionSLAM.__init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality, hole_width_mm)
            
        if not random_seed:
            random_seed = int(time.time()) & 0xFFFF
            
        self.randomizer = pybreezyslam.Randomizer(random_seed)

        self.sigma_xy_mm = sigma_xy_mm
        self.sigma_theta_degrees = sigma_theta_degrees
        self.max_search_iter = max_search_iter
    
    def _get_start_pos(self, map_size_meters):
        file_x = open("/var/lib/mercator/x","r")
        x = 1000 * float(file_x.read(10))
        file_x.close()
        file_y = open("/var/lib/mercator/y","r")
        y = 1000 * float(file_y.read(10))
        file_y.close()
        return pybreezyslam.Position(x, y, 0)
        
    def update(self, scan_mm, velocities=None):

        if not velocities:
        
            velocities = (0, 0, 0)
    
        CoreSLAM.update(self, scan_mm, velocities)    
    
    def _getNewPosition(self, start_position):
        '''
        Implements the _getNewPosition() method of SinglePositionSLAM. Uses Random-Mutation Hill-Climbing
        search to look for a better position and angle based on Visual Light Positioning by mercatord (http://github.com/kareldecoster/mercator).
        '''     
        file_x = open("/var/lib/mercator/x","r")
        file_y = open("/var/lib/mercator/y","r")
        try:
            x = 1000.00 * float(file_x.read(5))
            y = 1000.00 * float(file_y.read(5))
            start_position.x_mm = x
            start_position.y_mm = y
        except ValueError, e:
        #print "could not convert X Y to float\n"
            pass
        file_x.close()
        file_y.close()
        theta = self.position.theta_degrees
        print " Theta before RMHC = {theta}".format(theta=theta)
        # RMHC search is implemented as a C extension for efficiency
        return pybreezyslam.rmhcPositionSearch(
            start_position, 
            self.map, 
            self.scan_for_distance, 
            self.laser,
            self.sigma_xy_mm,
            self.sigma_theta_degrees,
            self.max_search_iter,
            self.randomizer)
                             
    def _random_normal(self, mu, sigma):
        
        return mu + self.randomizer.rnor() * sigma

