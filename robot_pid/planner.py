"""
   Path planner class implements the A*search algorithm which tracks the shorter path and allows the robot to traverse the shortest route to reach the goal.
   The Manhattan heuristic algorithm to compute how many blocks current robot's position be at distance from the goal position.

"""
#import map class
from map import Map
class AStarPlanner:

    """Plans shortest paths on a 4-connected Map using A*.

    Attributes:
        map_instance (Map): The occupancy grid the planner searches on.
    """
    def __init__(self,grid_map):
     """Store the map the planner will search on.

        Args:
            grid_map (Map): Occupancy grid to plan on.

        Raises:
            TypeError: If grid_map is not a Map instance.
        """

     #return an error message if grid_map is not an instance of Map()
     if not isinstance(grid_map,Map):
        raise TypeError('grid_map must be an object of the class Map')

     self.map_instance = grid_map
   


    @staticmethod
    def heuristic(a, b):
        """Manhattan distance between two (row, col) cells.

        Admissible and consistent on a 4-connected grid with step cost 1.

        Args:
            a (tuple): (row, col) of the first cell.
            b (tuple): (row, col) of the second cell.

        Returns:
            int: |a_row - b_row| + |a_col - b_col|.
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])



