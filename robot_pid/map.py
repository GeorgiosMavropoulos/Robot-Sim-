"""
Occupancy Grid Mapping Module for Robot Localization and Navigation.

This module provides a structured spatial environment ('Map' class) designed to interface 
discrete layout pathplanners (such as A* Search) with continuous real-world vehicle loops. 
It supports boundary guarding, collision checks, adjacent neighbor discovery, and 
bidirectional transformations between continuous metric coordinates and discrete grid spaces.

Classes:
    Map: Manages a 2D matrix array grid environment scaling indexes to real-world meters.

Methods inside Map:
    __init__(rows, cols, cell_size):
        Constructs the internal grid matrix array and initializes sizing dimensions.

    _validate_initialization_values(rows, cols, cell_size):
        [Static] Secure structural guard checking validation parameters during boot phases.

    _is_cell_valid_tuple(cell):
        Defensive type guard asserting input cells are properly structured (row, col) integer pairs.

    inbound_cell(cell):
        Evaluates structural indices determining if target locations sit within boundaries.

    is_free_cell(cell):
        Verifies if indices point to non-obstructed, navigatable positions.

    neighbors(cell):
        Finds adjacent traversable horizontal and vertical path options around an origin cell.

    cell_to_world(cell):
        Converts grid cell indices (row, col) to real-world center-point positions (x, y) in meters.
        
    world_to_cell(x, y):
        Converts robot odometer coordinate points (x, y) in meters to discrete grid slot rows and columns.
"""

import math
from typing import List
class Map:
    def __init__(self,rows,cols,cell_size=0.2):
        """Initializes the occupancy grid map.

        Args:
            rows (int): Total number of rows in the grid.
            cols (int): Total number of columns in the grid.
            grid is being created inside the __init__ and is not required to be included in the signature
            cell_size (float): Dimension of each cell side in meters. Defaults to 0.2.
        """

        #helper method to validate that input rows, cols are integers, grid dimensions are strictly positive numbers and cell size is a finite number
        self._validate_initialization_values(rows,cols,cell_size)

        self.rows = rows
        self.cols = cols
        self.grid = [[0] * cols for _ in range(rows)] #create the grid
        self.cell_size = cell_size #value is measured in meters.


    @staticmethod
    def _validate_initialization_values(rows,cols,cell_size):
       
       """Validates the structure and data types of the grid initialization parameters.

    This helper function acts as a guard mechanism during map creation. It 
    ensures that dimensions can form a valid matrix array and that spatial 
    scaling parameters are realistic, positive, and finite.

    Args:
        rows (int): Total number of horizontal rows intended for the grid.
        cols (int): Total number of vertical columns intended for the grid.
        cell_size (float): The metric scale representation of a single cell side in meters.

    Raises:
        TypeError: If either `rows` or `cols` is not an integer.
        ValueError: If `rows` or `cols` is less than or equal to zero, 
            or if `cell_size` is non-positive or a non-finite number (NaN/Inf).
    """
       if not isinstance(rows, int) or not isinstance(cols, int):
          raise TypeError("Grid dimensions (rows, cols) must be integers.")
       if rows <= 0 or cols <= 0:
             raise ValueError("Grid dimensions must be strictly positive numbers.")
       if not isinstance(cell_size, (int, float)) or cell_size <= 0 or not math.isfinite(cell_size):
            raise ValueError("Cell size must be a positive finite number.")


    #helper method to raise a type error if input cell is not a valid tuple
   
    def _is_cell_valid_tuple(self,cell):
          """Raise an error if cell is not a valid tuple with 2 values.Raise also an error if cell is not integer"""
          if not isinstance(cell, (tuple, list)) or len(cell) != 2:
                raise TypeError(f"Cell must be a tuple list with 2 values and both values must be integers. Your input is: {type(cell)}")
          #unpack the list since it's a tuple with 2 values
          row, col = cell

          #raise an error if both values inside the cell are not integers
          if not isinstance(row, int) or not isinstance(col, int):
            raise TypeError(f"Both values inside the cell must be integers. Received: row={type(row)}, col={type(col)}")
          

    

    #function to validate if rows,cols are inside the grid

    def inbound_cell(self,cell)->bool:
        """Checks if the given row and column indices are within the grid map boundaries.

        This method acts as a safety guard during runtime (e.g., inside planning 
        or simulation loops) to prevent IndexError crashes when checking specific cells.

        Args:
            cell (tuple): (row, col) indices of the cell to check

        Returns:
            bool: True if the coordinates are within the valid grid bounds, 
                False otherwise.
        """ 
        #raise a type error if cell is not a tuple with 2 values
        self._is_cell_valid_tuple(cell)
        #cell is a tuple with 2 values (rows,cols) so I break it into seperate variables
        row,col = cell

        
        #validate whether rows and cols are inside grid
        return 0 <= row < self.rows and 0 <= col < self.cols


    def is_free_cell(self,cell)->bool:

        """
           Uses inbound_cell method to validate if grid is inside bounds and returns False if it isn't.
           If cell is inside the grid it validates if this cell does not have an obstacle and the robot can freely move inside the cell.
           It acts as a safe guard so the robot won't collide with obstacles.

           Args: cell (tuple): (row, col) indices of the cell to check

           Returns:
                       bool: True if the grid is inside bounds and the robot can move to this grid, 
                           False otherwise.
           
        """
        #use in_bounds method to check if this cell is inside the grid
        if not self.inbound_cell(cell):
            return False
            
        
        #unpack the tuple into row and col
        row,col = cell

        read_cell = self.grid[row][col] #read cell's values

        #return true if cell's value is 0, this means it's free
        return read_cell == 0


    def neighbors(self,cell) -> List:
        """
        This functions loops through grid's directions and creates a neighbor cell. is_free_cell method validates whether a cell is free or not.
        If it's free it appends the cell into neighbors list.
        The method also raises an error if cell is not a valid tuple

        Args: 
          cell tuple (row, col) indices of the cell to check
          
           
        Returns:
            A list of tuples (row, col)
        
        """

        #raise a type error if cell is not a tuple with 2 values
        self._is_cell_valid_tuple(cell)

        #unpack cell to row and col
        row,col = cell

        #create a list to store the valid blocks the algorithm will find
        neighbors = [] 

        #the following list holds the direction's key. each pair represents a move. 
        # (0, 1)->right (same row next column), 
        # (0,-1)->left(same row, previous column),
        ## (1, 0)->(next row, same column), (-1,0)->(previous row, same column)
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)] 

        #loop through the list and for each direction create the neighbor cell adding the change in the row and col
        for direction in directions:
            r,c = row + direction[0], col + direction[1]

            #if method is free accepts the cell add it to the list (neighbors)
            if self.is_free_cell((r, c)):
                #append the coordinates to the list
                neighbors.append((r, c)) 

        return neighbors

        
    #convert cell's position into metres
    def cell_to_world(self,cell)->tuple:
      """
         Converts grid cell coordinates (row, col) into real-world metric coordinates (x, y).

        The A* planner works with grid indices, but the robot's physical layout operates
        on continuous meters. This method extracts the geometric center point of the cell.

        Additionally, this method returns an error message if cell is not a valid tuple 

        Args:
            cell (tuple): (row, col) matrix indices.

        Returns:
            tuple: (x, y) continuous positions in meters.
      
       """
     
      #raise a type error if cell is not a tuple with 2 values
      self._is_cell_valid_tuple(cell)
      #unpack the cell to row and col
      row,col = cell
      #finding row's center. +0.5 is being used to get the center and not the left edge
      x_center = (col + 0.5) * self.cell_size
      y_center = (row + 0.5) * self.cell_size

      #return x,y
      return (x_center,y_center)


    @staticmethod
    def is_x_y_valid(x,y):
            """Helper function to validate whether x,y are valid meter numbers and infite. If not raises an error"""
            if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                raise TypeError("X and Y values must be numbers.")
            
            #return an error message if coordinates are not fininte numbers
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError("Coordinates must be finite numbers!")


    #function to return the cell with a point in meters
    def world_to_cell(self,x, y)-> tuple:
        """This method returns the cell containg a mark to meters. In fact, it converts the meters (x,y) back to cell coordinates (row,col)
           Additionally, it raises a type rror if x,y are not intgers and if x,y are not finite numbers.
           Finaly this method calculates row and col coordinates and uses math.floor() to round to closest small integer.
        Args:
           x col's center
           y row's center

        Returns: 
             cell tuple (col, row)
        """

        #return an error message if x,y are not  integers and not finite numbers
        self.is_x_y_valid(x,y)
        
        
        #find row's coordinates
        row = math.floor(y / self.cell_size) #round the float number to make it an integer

        #find col's coordinates
        col = math.floor(x / self.cell_size) ##round the float number to make it an integer

        #create a tuple containing col, row
        cell = row,col
        return cell



    
    def is_free(self,x,y)-> bool:
        """
        This function will be used from the simulator to count collisions. It answers whether the robot can pass the current block.
        This method uses world_to_cell to convert x,y to coordinates and the method is_free_cell for the specific cell and returns the answer
        
        Args: 
                x (col)
                y (row)

        Returns:
                True if cell is free
                False if not
        
        """
        #return an error message if x,y are not  integers and not finite numbers
        self.is_x_y_valid(x,y)

        #convert x,y to coordinates
        cell = self.world_to_cell(x,y)

        #call is_free_cell method to validate if this cell is free or not
        is_free = self.is_free_cell(cell)

        return is_free




    def add_obstacle_rect(self,r0, c0, r1, c1):
        """
         This method fills a rectangle of cells with 1. (r0,c0) is one edge and (r1,c1) the opposite edge
         Both angles belong to the rectangle. Both numbers must be integers and both edge inside the map.
         The inbound_cell method is being used to validate the edge. 
         Angle order: if user inputs r0 > r1 or c0 > c1, the method swap the values in order not to raise an error. 
         The user now does not care about the order.

         Args:
             r0, c0, r1, c1 -> integers which represent the edge of the rectangle
             r0,c0: row and column of an edge
             r1,c1: row and column of the other edge

        Returns:
                    None
                
        
        """

        #validate if edges are inside the map boundaries and they are integers. inbound_cell verifies if they integers too!
        if not self.inbound_cell((r0, c0)) or not self.inbound_cell((r1, c1)):
            raise TypeError(f"Both edges ({r0}, {c0}) and ({r1}, {c1}) must be inside the map boundaries.")

        #swap the values if users inputs r0 > r1 or c0> c1
        if r0 >r1:
            r0, r1 = r1, r0

        if c0 > c1:
            c0,c1 = c1,c0        

        #if both angles are valid fill the rectangle with 1 using a nested for loop
        for row in range(r0, r1 + 1):
            for col in range(c0, c1 + 1):
                self.grid[row][col] = 1


        

        
             
