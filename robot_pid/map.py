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
          """Raise an error if cell is not a valid tuple with 2 values"""
          if not isinstance(cell, (tuple, list)) or len(cell) != 2:
                raise TypeError(f"Cell must be a tuple list with 2 values. Your input is: {type(cell)}")
          

    

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


    def is_free_cell(self,cell)->int:

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
    def cell_to_world(self,cell):
      """
         Converts grid cell coordinates (row, col) into real-world metric coordinates (x, y).

        The A* planner works with grid indices, but the robot's physical layout operates
        on continuous meters. This method extracts the geometric center point of the cell.

        Additionally, this method returns an error message if cell is not a valid tuple 

        Args:
            cell (tuple): (row, col) matrix indices.

        Returns:
            tuple: (x, y) continuous positions in meters.

       Returns:
            
      
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


m = Map(5, 5)
print(m.cell_to_world((1, 2)))