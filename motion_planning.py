import argparse
import time
import msgpack
from enum import Enum, auto
import numpy as np

from planning_utils import a_star, heuristic, create_grid, collinearity_prune, bresenham_prune, a_star_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # read lat0, lon0 from colliders into floating point values
        filename = 'colliders.csv'
        home = np.genfromtxt(filename, dtype=str, max_rows=1, delimiter=',')
        lat0 = float(str.split(home[0])[1])
        lon0 = float(str.split(home[1])[1])

        # set home position to (lon0, lat0, 0)
        self.set_home_position( float(lon0), float(lat0), 0)
        
        # convert to current global position to local position using global_to_local()
        local_pose = global_to_local(self.global_position, self.global_home) 
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # convert start position to current position rather than map center
        grid_start = ( int(local_pose[0]) - north_offset, int(local_pose[1]) - east_offset)
 
        # Set goal using provided global position
        goal_lat_lon = ( args.goal_lon, args.goal_lat, args.goal_alt )
        local_goal = global_to_local( goal_lat_lon, self.global_home )
        grid_goal = ( int(local_goal[0]) - north_offset, int(local_goal[1]) - east_offset )

        # make sure goal point is not inside an obstacle or safety margin
        if (grid[grid_goal[0]][grid_goal[1]] == 1):
            print("Invalid goal point: Aborting")
            self.landing_transition()
            return
     
        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        # Prune path to minimize number of waypoints
        if (args.prune == 'collinearity'):
            print('Pruning path with collinearity check')
            path = collinearity_prune(path)
        elif (args.prune == 'bresenham'):
            print('Pruning path with bresenham')
            path = bresenham_prune(path,grid)
        else:
            print('Unrecognized prune option returning full path')

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]

        # Assign heading to the waypoints so the craft flies pointed towards the waypoint 
        for i in range(1, len(waypoints)):
            p1 = waypoints[i-1]
            p2 = waypoints[i]
            waypoints[i][3] = np.arctan2((p2[1]-p1[1]), (p2[0]-p1[0]))

        # Set self.waypoints
        self.waypoints = waypoints

        # send waypoints to sim (this is just for visualization of waypoints)
        print("Waypoints:", len(self.waypoints))
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--prune', type=str, default='bresenham', help="Path pruning Algorithm ('collinearity' or 'bresenham'")
    parser.add_argument('--goal_lon', type=float, default=-122.397120, help='Goal latitude')
    parser.add_argument('--goal_lat', type=float, default=37.793834, help='Goal longitude')
    parser.add_argument('--goal_alt', type=float, default=0, help='Goal altitude')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
