import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):
    # https://github.com/udacity/udacidrone/blob/master/udacidrone/drone.py

    def __init__(self, connection):
        super().__init__(connection)
        self.flight_state = States.MANUAL

        self.box_size = 10
        self.box_altitude = 3

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = self.calculate_box()
        self.in_mission = True

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        '''
        Triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        '''
        if self.flight_state == States.TAKEOFF:
            if self._altitude > 0.95 * self.target_position[2]:
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            if (abs(self.target_position[0] - self.local_position[0]) < .25 and
                abs(self.target_position[1] - self.local_position[1]) < .25):
                if self.all_waypoints:
                    self.waypoint_transition()

                elif np.linalg.norm(self.local_velocity) < .5:
                    self.landing_transition()

    def velocity_callback(self):
        '''
        Triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        '''
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < .25:
                self.disarming_transition()

    def state_callback(self):
        '''
        Triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        '''
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()

        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()

        elif self.flight_state == States.DISARMING:
            if not self.armed and not self.guided:
                self.manual_transition()

    def calculate_box(self):
        return [
            [0, 0, self.box_altitude],
            [0, self.box_size, self.box_altitude],
            [self.box_size, self.box_size, self.box_altitude],
            [self.box_size, 0, self.box_altitude],
        ]

    def arming_transition(self):
        print("arming transition")
        self.flight_state = States.ARMING
        self.set_home_position(
            self.global_position[0],
            self.global_position[1],
            self.global_position[2]
        )
        self.take_control()
        self.arm()

    def takeoff_transition(self):
        print("takeoff transition")
        self.flight_state = States.TAKEOFF
        self.target_position[2] = self.box_altitude
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        print("waypoint transition")
        self.flight_state = States.WAYPOINT
        self.target_position = self.all_waypoints.pop()
        print('heading to {}'.format(self.target_position))
        self.cmd_position(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            0
        )

    def landing_transition(self):
        print("landing transition")
        self.flight_state = States.LANDING
        self.land()

    def disarming_transition(self):
        print("disarm transition")
        self.flight_state = States.DISARMING
        self.disarm()
        self.release_control()

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        super().start()

        self.stop_log()


if __name__ == "__main__":
    connection = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = BackyardFlyer(connection)
    time.sleep(2)
    drone.start()
