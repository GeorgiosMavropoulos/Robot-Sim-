"""
Occupancy Grid Mapping Module for Robot Localization and Navigation.

This module provides a structured spatial environment ('Map' class) designed to interface 
discrete layout path planners (such as A* Search) with continuous real-world vehicle loops. 
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
        [Static] Defensive type guard asserting input cells are properly structured (row, col) integer pairs.

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

    add_obstacle_rect(r0, c0, r1, c1):
        Fills a rectangle array space of cells between two designated corner coordinates with obstacles.

    build_default_map():
        [Classmethod] Factory constructor generating a benchmark slalom path testing environment.
"""

import math
from typing import List
class Map:
    def __init__(self,rows,cols,cell_size=0.2):
        """Create an empty map (all cells free).

        Args:
            rows (int): Number of rows, > 0.
            cols (int): Number of columns, > 0.
            cell_size (float): Cell side in metres, positive and finite.
                Defaults to 0.2.

        Raises:
            TypeError: If rows or cols is not an integer.
            ValueError: If rows or cols <= 0, or cell_size is not a
                positive finite number.
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
    @staticmethod
    def _is_cell_valid_tuple(cell):
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

        """Tell whether a cell is inside the map and free of obstacles.

            Args:
                cell (tuple): (row, col) indices.

            Returns:
                bool: True if the cell is inside the grid and holds no obstacle.
                A cell outside the grid is not free, so False is returned
                instead of raising an error (the planner asks about neighbours
                at the map border).

            Raises:
                TypeError: If cell is not a pair of integers.
            """
        #use in_bounds method to check if this cell is inside the grid
        if not self.inbound_cell(cell):
            return False
            
        
        #unpack the tuple into row and col
        row,col = cell

        read_cell = self.grid[row][col] #read cell's values

        #return true if cell's value is 0, this means it's free
        return read_cell == 0


    def neighbors(self,cell) -> List[tuple]:
        """Return the free 4-connected neighbours of a cell.

        Neighbours are checked in the order right, left, down, up
        (row +1 is down, row -1 is up).

        Args:
            cell (tuple): (row, col) indices.

        Returns:
            list[tuple]: (row, col) of every free neighbour; cells outside
            the map or holding an obstacle are left out.

        Raises:
            TypeError: If cell is not a pair of integers.
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

        Additionally, this method raises a TypeError if cell is not a valid tuple 

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
            """Check that x and y are finite numbers.

        Raises:
            TypeError: If x or y is not an int or float.
            ValueError: If x or y is NaN or infinite.
         """
            
            if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                raise TypeError("X and Y values must be numbers.")
            
            #return an error message if coordinates are not fininte numbers
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError("Coordinates must be finite numbers!")


    #function to return the cell with a point in meters
    def world_to_cell(self,x, y)-> tuple:
        """Convert a position in metres to the cell that contains it.

        Uses math.floor, so a point just left of or above the map lands
        in a cell with a negative index (outside the grid) instead of
        being rounded into cell 0.

        Args:
            x (float): Horizontal position in metres (-> column).
            y (float): Vertical position in metres (-> row).

        Returns:
            tuple: (row, col). May lie outside the grid.

        Raises:
            TypeError: If x or y is not a number.
            ValueError: If x or y is not finite.
        """

        #return an error message if x,y are not  integers and not finite numbers
        self.is_x_y_valid(x,y)
        
        
        #find row's coordinates
        row = math.floor(y / self.cell_size) #round the float number to make it an integer

        #find col's coordinates
        col = math.floor(x / self.cell_size) ##round the float number to make it an integer

        #create a tuple containing row,col
        cell = row,col
        return cell



    
    def is_free(self,x,y)-> bool:
        """Collision check for a point given in metres.

        Args:
            x (float): Horizontal position in metres.
            y (float): Vertical position in metres.

        Returns:
            bool: True if the point lies inside the map on a free cell.
            Points outside the map return False.

        Raises:
            TypeError, ValueError: As in world_to_cell.
        """

        #convert x,y to coordinates and also validates if x,y are valid numbers
        cell = self.world_to_cell(x,y)

        #call is_free_cell method to validate if this cell is free or not
        is_free = self.is_free_cell(cell)

        return is_free




    def add_obstacle_rect(self,r0, c0, r1, c1):
        """Fill a rectangle of cells with obstacles.

        (r0, c0) and (r1, c1) are opposite corners and both belong to the
        rectangle. The corners may be given in any order; they are swapped
        if needed.

        Args:
            r0, c0 (int): Row and column of one corner.
            r1, c1 (int): Row and column of the opposite corner.

        Raises:
            TypeError: If a corner is not a pair of integers.
            ValueError: If a corner lies outside the map.
        """

        #validate if edges are inside the map boundaries and they are integers. inbound_cell verifies if they integers too!
        if not self.inbound_cell((r0, c0)) or not self.inbound_cell((r1, c1)):
            raise ValueError(f"Both edges ({r0}, {c0}) and ({r1}, {c1}) must be inside the map boundaries.")

        #swap the values if users inputs r0 > r1 or c0> c1
        if r0 >r1:
            r0, r1 = r1, r0

        if c0 > c1:
            c0,c1 = c1,c0        

        #if both angles are valid fill the rectangle with 1 using a nested for loop
        for row in range(r0, r1 + 1):
            for col in range(c0, c1 + 1):
                self.grid[row][col] = 1



    @classmethod
    def build_default_map(cls):
       """Creates a benchmark occupancy grid map with a stable layout of 3 obstacles.
        
        The obstacles are placed as interlocking walls to force any path-planning 
        algorithm (like A*) to compute a trajectory with distinct left and right turns. 
        This ensures all experiments compare tracking and controller gains with fairness.

        Args:
            None

        Returns:
            Map: An initialized Map instance with predefined static wall obstacles.
        """
       #declare a default map 20 * 20 with cell_size 0.2
       default_map = cls(rows=20, cols=20, cell_size=0.2)

       #build 3 wall-obstacles using add_obstacle_rect method
      
        #obstacle 1: Wall starting from the left edge and closes to 70% of the width
       default_map.add_obstacle_rect(r0=5, c0=0, r1=5, c1=13)

        #obstacle 2: A wall which starts from the right edge and closes the 70% of the width
       default_map.add_obstacle_rect(r0=10, c0=6, r1=10, c1=19)

       #obstacle 3: another wall from the left side
       default_map.add_obstacle_rect(r0=15, c0=0, r1=15, c1=13)

       return default_map

        

        
             
m = Map.build_default_map()
for row in m.grid:
    print("".join("#" if v else "." for v in row))
print(sum(sum(row) for row in m.grid))                      # 42 (3 τοίχοι × 14 κελιά)
print(m.is_free_cell((0, 0)), m.is_free_cell((19, 19))) 