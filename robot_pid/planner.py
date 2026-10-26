"""
   Path planner class implements the A*search algorithm which tracks the shorter path and allows the robot to traverse the shortest route to reach the goal.
   The Manhattan heuristic algorithm to compute how many blocks current robot's position be at distance from the goal position.

"""
#import map class
from map import Map
import math
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

    ##Error handling#####

    #validate that a and b are tuples or lists of length 2
        for point, name in [(a, "a"), (b, "b")]:
         if not isinstance(point, (tuple, list)) or len(point) != 2:
            raise ValueError(
                f"{name} must contain exactly two coordinates (row, col)"
            )

        #validate that both coordinates are numeric and finite
        for coordinate in point:
            if (
                isinstance(coordinate, bool)
                or not isinstance(coordinate, (int, float))
                or not math.isfinite(coordinate)
            ):
                raise ValueError(
                    f"Coordinates in {name} must be finite numbers"
                )

        ##apply the manhattan algorithm if all goes well  
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def reconstruct_path(came_from,goal):
       """
            Reconstruct the path from start to goal.

            Args:
                came_from (dict): Maps each cell to its predecessor.
                goal (tuple): Target cell.

            Returns:
                list: Cells ordered from start to goal.
        """


       #validate came_from
       if not isinstance(came_from, dict):
        raise TypeError("came_from must be a dictionary")

        #validate goal structure
       if not isinstance(goal, tuple) or len(goal) != 2:
        raise ValueError("goal must be a tuple of two integers")

       #validate goal coordinates
       if any(type(coord) is not int for coord in goal):
        raise ValueError("goal coordinates must be integers")

       #create an empty list to append the coordinates in order to designate the path
       path = [] 
       current = goal

       #loop through came_from list and append current into the path
       while current in came_from:
           path.append(current)
           current = came_from[current]

       path.append(current)  # Add the start cell
       path.reverse() #reverse the list since it starts from the goal cell to designate the path

       return path


grid_item = Map(5, 5)
planner = AStarPlanner(grid_item)

came_from = {
    (1, 2): (1, 1),
    (1, 3): (1, 2),
    (2, 3): (1, 3),
    (3, 3): (2, 3)
}

goal = (1, 2)

path = planner.reconstruct_path(came_from, goal)
print(path)
