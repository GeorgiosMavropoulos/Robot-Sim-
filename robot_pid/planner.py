"""A* path planner used by the robot to find a route on the occupancy grid.

Design rationale
----------------
The planner works on the discrete grid of ``Map`` and returns cells
(row, col). The conversion to metres is a separate step, so the search
itself can be tested with exact integer comparisons, while the simulator
and the PID controllers only ever see (x, y) waypoints in metres.

* Moves are 4-connected (right, left, down, up) and every step costs 1.
* The heuristic is the Manhattan distance, which never overestimates the
  real cost on this grid (admissible) and is consistent, so the path
  found is a shortest one.
* The open set is a ``heapq`` priority queue ordered by f = g + h.
  ``heapq`` cannot lower the priority of an entry, so a cell may be pushed
  more than once; outdated entries are skipped when they are popped.
* Neighbours come from ``Map.neighbors`` in a fixed order, so the same
  map, start and goal always give the same path.

Public API
----------
AStarPlanner(grid_map)
    Create a planner for a ``Map``. Raises TypeError if grid_map is not
    a ``Map``.
AStarPlanner.plan(start, goal) -> list[tuple] or None
    Shortest path as (row, col) cells, start and goal included. Returns
    None if the goal cannot be reached. Raises ValueError if start or
    goal is outside the map or on an obstacle.
AStarPlanner.get_waypoints(start, goal) -> list[tuple] or None
    Same search, but the path is returned as (x, y) positions in metres
    (cell centres). This is what the simulator calls.

Helpers
-------
AStarPlanner.heuristic(a, b) -> int
    Manhattan distance between two cells (static method).
AStarPlanner.reconstruct_path(came_from, goal) -> list[tuple]
    Rebuilds the path by following came_from back from the goal and
    reversing it (static method).

Coordinate convention
---------------------
Cells are (row, col). Positions in metres are (x, y), where x corresponds
to the column and y to the row.
"""
#import map class
from .map import Map
import math
import heapq
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
            
            if not isinstance(point, (tuple, list)):
                raise TypeError(
                    f"{name} must be a tuple or a list, got {type(point).__name__}"
                )
                
            
            if len(point) != 2:
                raise ValueError(
                    f"{name} must contain exactly two coordinates, got length {len(point)}"
                )

            for coordinate in point:
                #Raise TypeError if coordinates are not int or float
                if isinstance(coordinate, bool) or not isinstance(coordinate, (int, float)):
                    raise TypeError(
                        f"Coordinates in {name} must be numeric (int or float), got {type(coordinate).__name__}"
                    )
            
            #raise a ValueError if coordinates are not finite numbers
            if not math.isfinite(coordinate):
                raise ValueError(
                    f"Coordinates in {name} must be finite numbers, got {coordinate}"
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
        raise ValueError("came_from must be a dictionary")

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


    ##plan method
    def plan(self,start,goal):
       """
       Find the shortest 4-connected path between two cells using A*.

        Searches the occupancy grid with a priority queue ordered by
        f = g + h, where g is the number of steps from start (each step
        costs 1) and h is the Manhattan heuristic to the goal. Outdated
        queue entries are skipped. Neighbours are explored in the fixed
        order given by Map.neighbors, so the same inputs always produce
        the same path.

        Args:
            start (tuple): (row, col) of the starting cell. Must be a
                tuple of ints, inside the map and free.
            goal (tuple): (row, col) of the target cell. Same rules as start.

        Returns:
            list[tuple] or None: Cells (row, col) ordered from start to
            goal, both included. If start == goal, a list with that single
            cell. None if the goal cannot be reached from start.

        Raises:
            ValueError: If start or goal lies outside the map or on an
                obstacle (invalid input, as opposed to "no path").
            TypeError: If start or goal is not a pair of integers
                (raised by Map.is_free_cell).
        """

       #validate that the start and goal cells are inside the map
       # and are not occupied by obstacles.
       if not self.map_instance.is_free_cell(start):
        raise TypeError("Start cell must be inside the map and free.")

       if not self.map_instance.is_free_cell(goal):
        raise TypeError("Goal cell must be inside the map and free.")

       #convert start into a tuple list because g_score requires a hashable key
       start = tuple(start)

       #priority queue containing (estimated total cost, cell).
       open_list = []

       #this list stores the predecessor of each visited cell to reconstruct
       # the final path.
       came_from = {}

       #g_score stores the lowest known cost from start to each cell.
       g_score = {start: 0}

       #f_score estimates the total cost: g_score + heuristic.
       f_score = {
            start: self.heuristic(start, goal)
        }

       #add the starting cell to the priority queue
       heapq.heappush(open_list,(f_score[start], start))

       #continue searching while there are cells left to explore.
       while open_list:
          #remove the cell with the lowest estimated total cost
          current_f, current = heapq.heappop(open_list) 

          #skip outdated entries whose cost is worse than the
          # best estimate currently recorded for this cell.
          if current in f_score and current_f > f_score[current]:
            continue

          #if the goal is reached, reconstruct and return the path.
          if current == goal:
             #call reconstruct_path method to map the shorter found path
             path = self.reconstruct_path(came_from,goal)
             return path

          #explore all free neighbouring cells.
          for neighbor in self.map_instance.neighbors(current):
             
             #moving to an adjacent cell costs one step.
             temp_g = g_score[current] + 1

             #update the neighbour only if this route is better
             # than any previously discovered route.
             if temp_g < g_score.get(neighbor, float("inf")):
                #record the cell from which this neighbour was reached.
                came_from[neighbor] = current

                #store the improved cost from start.
                g_score[neighbor] = temp_g

                #estimate the total cost through this neighbour.
                f_score[neighbor] = (
                    temp_g + self.heuristic(neighbor, goal)
                )

                #add the neighbour to the priority queue
                heapq.heappush(
                    open_list,
                    (f_score[neighbor], neighbor)
                )
       #the goal cannot be reached from the start.
       return None


    #function to convert rows,cols (returned path from plan) to x,y since the robot can only understand x,y
    def get_waypoints(self,start,goal):

       """
        Find a path and convert its cells into world coordinates.

        Calls plan method from this class to find the shortest path.

        Calls cell_to_world method from Map class to convert row and col into x,y.
        
        Args:
            start (tuple): Starting cell (row, col).
            goal (tuple): Goal cell (row, col).

        Returns:
            list[tuple] or None: Waypoints as (x, y) coordinates,
            or None if no path is found.

        Raises:
          ValueError: If start or goal lies outside the map or on an
                        obstacle (invalid input, as opposed to "no path").
                    TypeError: If start or goal is not a pair of integers
                        (raised by Map.is_free_cell).
                        
                
        """

       #call plan method to retrieve the path
       path = self.plan(start,goal)

       if path is None:
        return None

       waypoints = []
       #iterate through path and convert each row and col into x,y
       for p in path:       
       ##call the method cell_to_world from Map class to convert the rows and cols into x,y
        waypoints.append(self.map_instance.cell_to_world(p))
           

       return waypoints





