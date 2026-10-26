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


map = Map(3,3)
print(map.inbound_cell((-1, -1)))


