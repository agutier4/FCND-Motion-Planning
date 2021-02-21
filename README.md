## Project: 3D Motion Planning
### Andrew Gutierrez, 2/20/21
![Quad Image](./misc/motion_planning.png)
Motion planning in action with path pruned using the Bresenham algorithm

---

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

---
# Writeup
## Starter Code Explained
The starter code contained in `motion_planning.py` and `planning_utils.py` provides much of the framework for getting the drone up and running with basic waypoint functionality. Much of the basic event-driven control code provided is similar to the previous project and the solution provided in `backyard_flyer_solution.py`. This defines how the drone transitions through the various states and sets up the callbacks to watch the important state information to propperly transition through the mission. Unlike the previous project, there is now a `plan_path()` function to generate the waypoints that outline the flight path instead of the calculate box function. This is where the search problem is setup and performed, eventhough much of the important utility functions live in `planning_utils.py`. 

One utility is `create_grid()` which is used to generate the grid representation of the environment from the obstacle data contained in `colliders.csv`. The Action class creates an object representation of motions the craft can make and including some basic utilities used in the planning process. `valid_actions()` generates a list of all possible actions that are available to the vehicle at a given location on a given map. This is used to expand the nodes of the grid in search. Finally `a_star()` is probably the most important of the utilities as it is the core algorithm used in the planning process. It iterates through the search space and expands the nodes it thinks are most likely to lead to the goal first based on the cost to reach the current point and the heuristic which estimates the remaining cost. The heuristic defined here uses the straight line distance, since this is admissible, we can assume our planning will always generate the optimal path to the goal.

---

## Implementation Writeup
The implementation for the required elements are listed and described below

### 1. Set your global home position
The global home is setup in `motion_planning.py` specifically in the begining of `plan_path()` around line 120. The home position lat/lon values are parsed from the first first line of `colliders.csv` using NumPy utilites and written the the drone as the home referece position.

### 2. Set your current local position
After the global home reference is established, the current global position member values are taken and converted to the local reference frame using the `global_to_local()` function on line 130 of `motion_planning.py`.

### 3. Set grid start position from local position
The local position value calculated in the previous step is then used to create the grid start position. Because the grid is discritized, the floating point local position needs to first converted to an integer index and the offset by the grid offset to get the effective starting grid index. This process is done for both the north and east components.

### 4. Set grid goal position from geodetic coords
Currently, the code is setup to parse the goal position as program arguements in the form `--goal_lat=<value> --goal_lon=<value> --goal_alt=<value>`. If no arguements are provided, there is a fairly central default goal that is loaded. It should be noted that the `goal_alt` value will default to zero and can be ommited. The goal is then converted to local coordiantes, again, using `global_to_local()` and then converted to a set of grid indecies using the same process as the start. This position is then verified against the grid representation to make sure the goal is not within a building or it's safety margin. If the goal is invalid, the drone will abort the plannign process and transition to landing.

### 5. Modify A* to include diagonal motion (or replace A* altogether)
The base A* has been modified to include diagonal motion at a cost of sqrt(2). Tha beign said, this did not actually involve any changes to `a_star()` only to the Action class. Ordinal directions were added to the `Action` class with the approriate weights along with the appropriate checks in `valid_actions()`. This is what is called within the A* implementation during the expansion step and it determines if the diagonal actions are possible.

### 6. Cull waypoints 
The code is setup to cull waypoints with either a collinearity check or with ray-tracing with the Bresenham Algorithm. The default is set to use Bresenham as it generally does a better job of cutting down on the number of waypoints. This is because the collinearity check only looks to see if the points lie on the same line wheras Bresenham actually checks to see if the waypoint actually serves a purpose. The path pruning algorithm can be set as a program arguement in the form `--prune=collinearity`. If an invalid pruning option is provided, the waypoints will not be culled in any way resulting in a lot of acceleration and deceleration to hit the many waypoints.

## Execute the flight
### 1. Does it work/Simulator bugs
Yes the planner does work. There are a number of bugs in the simulator that can potential cause issues. For instance, the drone currently spawns in the middle of a building and will clip up through the ceiling of it on takeoff in the most recent version of the simulator. This can be avoided by moving manually to a new start location before running the planning or by running an older version of the simulator. This will unfortunatly break the heading commands outlined below. In addition, there is an issue in the drone comms where if the planning process takes too long, the sim will reject the waypoints even if they are generated correcly.. I have verified that the planning is still succesful by logging these longer paths and loading them in as a manual plan so this is not an issue with the planner. There is not much I can do to avoid this short of editing the udacidrone source code so it is best to send goal positions that are not on the furthest corner of the map to prevent this from happening. If it does the error will take the form of:

`File "/home/<user>/miniconda3/envs/fcnd/lib/python3.6/site-packages/udacidrone/drone.py", line 117, in on_message_receive
    if (((msg.time - self._message_time) > 0.0)):
AttributeError: 'int' object has no attribute 'time'`
  
## Extra Challenges:

### 1. Heading Waypoints
![Quad Image](./misc/heading.png)
If the motion planning routine is run in the latest version of the sim, it will also generate headings for the waypoints. This is done by with a simple angle calculation between each waypoint and the next. It allows the drone to fly directly towards each waypoint as shown above instead of always facing the same direction and moving laterally towards the goal.