import math
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

    

    #function to validate if rows,cols are inside the grid

    def inbound_cell(self,cell):
        """Checks if the given row and column indices are within the grid map boundaries.

        This method acts as a safety guard during runtime (e.g., inside planning 
        or simulation loops) to prevent IndexError crashes when checking specific cells.

        Args:
            cell (tuple): (row, col) indices of the cell to check

        Returns:
            bool: True if the coordinates are within the valid grid bounds, 
                False otherwise.
        """

        #cell is a tuple with 2 values (rows,cols) so I break it into seperate variables
        row,col = cell

        
        #validate whether rows and cols are inside grid
        return 0 <= row < self.rows and 0 <= col < self.cols


    def is_free_cell(self,cell):

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


    def neighbors(self,cell):
        """
        This functions loops through grid's directions and creates a neighbor cell. is_free_cell method validates whether a cell is free or not.
        If it's free it appends the cell into neighbors list

        Args: 
          cell tuple (row, col) indices of the cell to check
          
           
        Returns:
            A list of tuples (row, col)
        
        """

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

        




m = Map(5, 5)
m.grid[0][1] = 1
tests = [
    ((0, 0), True),
    ((0, 1), False),
    ((2, 2), True),
    ((3, 0), False),
    ((-1, 0), False),
    ((0, 3), False),
]
for cell, expected in tests:
    result = m.neighbors(cell)
    print(cell, result, "OK" if result == expected else "FAIL")