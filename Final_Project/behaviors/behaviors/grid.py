import numpy as np
from nav_msgs.msg import OccupancyGrid
from copy import copy

class grid2D:
    # Start with a ros occupancy grid message
    def __init__(self, map: OccupancyGrid):
        self.num_rows=map.info.height
        self.num_cols=map.info.width
        self.map=np.reshape(map.data,(self.num_rows,self.num_cols))
        self.resolution=map.info.resolution
        self.half_res=self.resolution/2.0
        self.minXY=np.array([map.info.origin.position.x-self.half_res, map.info.origin.position.y-self.half_res])
        self.maxXY=self.minXY + np.array([self.num_cols*self.resolution, self.num_rows*self.resolution])

    # Check if the input coordinates are in bounds
    def is_xy_in_bounds(self, X, Y):
        if X<self.minXY[0] or X>self.maxXY[0]:
            return False
        if Y<self.minXY[1] or X>self.maxXY[1]:
            return False
        return True

    def is_rc_in_bounds(self, row, col):
        if row<0 or row>=self.num_rows:
            return False
        if col<0 or col>=self.num_cols:
            return False
        return True

    # Transform between XY and RC spaces
    def xy_to_rc(self, X, Y):       
        col=np.floor((X-self.minXY[0])/self.resolution).astype(int)
        row=np.floor((Y-self.minXY[1])/self.resolution).astype(int)
        return row, col
    
    def rc_to_xy(self, row, col):
        y=self.minXY[1]+row*self.resolution + self.half_res
        x=self.minXY[0]+col*self.resolution + self.half_res
        return x,y
    
    # Visualization utilities
    def get_ranges(self):
        xx=np.arange(self.minXY[0]+self.half_res,self.maxXY[0],self.resolution)
        yy=np.arange(self.minXY[1]+self.half_res,self.maxXY[1],self.resolution)
        return xx,yy
    
    def visualize_grid(self):
        import matplotlib.pyplot as plt
        xx,yy=self.get_ranges()
        plt.contourf(xx,yy,self.map)
        plt.show()

    # Get value functions
    def get_value_rc(self, row, col):
        if self.is_rc_in_bounds(row,col):
            return self.map[row,col]

    def get_value_xy(self, X, Y):
        r,c=self.get_row_col(X,Y)
        return self.get_value_rc(r,c)
    
    def create_occupancy_grid(self, min_value=-255, max_value=255):
        # This function
        map=OccupancyGrid()
        map.info.height=self.num_rows
        map.info.width=self.num_cols
        map.info.resolution=self.resolution        
        map.info.origin.position.x=self.minXY[0]+self.half_res
        map.info.origin.position.y=self.minXY[1]+self.half_res

        # Need to convert map data to [0,100]
        map.data=100.0*(np.reshape(self.map,(self.num_rows*self.num_cols))-min_value)/(max_value-min_value)
        map.data=np.where(map.data<0,0,map.data)
        map.data=np.where(map.data>100,100,map.data)
        map.data=map.data.astype(np.int8).tolist()
        return map
