import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv
import sys

# Importing planning_utils
from my_planning_utils_graph import a_star, heuristic, create_grid, create_grid_and_edges, show_start_goal, closest_point, collinearity_int
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

#Two different reference frames are defined and used within the Drone API. Global positions are defined as [longitude, latitude, altitude (positive up)]. Local reference frames #are defined [North, East, Down (positive down)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame. The #global reference frame is what is provided by the Drone's GPS. Two convenience functions, global_to_local() and local_to_global() are provided within the frame_utils.py script #to convert between the two frames. These functions are wrappers on utm library functions.

from udacidrone.frame_utils import global_to_local

# Numbering performed automatically (concrete value not important)
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

        self.target_position = np.array([0.0, 0.0, 0.0])
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

        # TODO: read lat0, lon0 from colliders into floating point values
        lat, lon =  open('colliders.csv').readline().split(",")
        lat0 = float(lat.strip("lat0 "))
        lon0 = float(lon.strip("lon0 "))

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        self.global_position = (self._latitude, self._longitude, self._altitude)

        # TODO: convert to current local position using global_to_local()
        self.local_position = self.global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # TODO: adapt create_grid_and_edges to return north_offset and east_offset
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset, edges = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print(len(edges))

        # create graph from edges
        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = LA.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (self.local_position[0] - north_offset, self.local_position[1] - east_offset)
        # TODO: convert start position to current position rather than map center


        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10) (Initial)
        # TODO: adapt to set goal as latitude / longitude position and convert
        try:
            goal_lon, goal_lat = float(input("Please provide Lon/Lat coordinates separated by ','.").split(","))
        except ValueError:
            print("Please insert Lon/Lat as a float")
            sys.exit()

        global_goal = (goal_lon, goal_lat, 0)
        local_goal = global_to_local(global_goal, self.global_home)
        grid_goal = (local_goal[0] -north_offset, local_goal[1] -east_offset)
        print('Grid Goal: ', grid_goal)

        # conversion for graph
        start_ne_g = closest_point(G, grid_start)
        goal_ne_g = closest_point(G, grid_goal)
        print(start_ne_g)
        print(goal_ne_g)


        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        # Graph solution choosen
        print('Local Start and Goal: ', start_ne_g, goal_ne_g)
        path, cost = a_star(G, heuristic, start_ne_g, goal_ne_g)
        print(len(path))

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        for p in range(len(path)-2):
            if collinearity(path[p],path[p+1],path[p+2]):
                path.remove(p+1)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

        # visulation from Graph search
        plt.imshow(grid, origin='lower', cmap='Greys')

        for e in edges:
            p1 = e[0]
            p2 = e[1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

        plt.plot([start_ne[1], start_ne_g[1]], [start_ne[0], start_ne_g[0]], 'r-')
        for i in range(len(path)-1):
            p1 = path[i]
            p2 = path[i+1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
        plt.plot([goal_ne[1], goal_ne_g[1]], [goal_ne[0], goal_ne_g[0]], 'r-')

        plt.plot(start_ne[1], start_ne[0], 'gx')
        plt.plot(goal_ne[1], goal_ne[0], 'gx')

        plt.xlabel('EAST', fontsize=20)
        plt.ylabel('NORTH', fontsize=20)
        plt.show()

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
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
