"""
TRAVEL MODE
-----------

Goes forward until the robot meets an obstacle in front.
Then it moves to one orthogonal direction.
Then it start traveling forward again.

"""
import logging
import time
from threading import Timer
import math
import numpy as np

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M'
THRESHOLD = 0.4 # m 
VELOCITY = 0.2
OFFSET_TIME = 0.3


TRAVEL_STATE = "travel"
SEARCH_STATE = "search"
LANDING_STATE = "landing"

W_LR = 1 # 1.5
H_LR = 1 # 3
OFFSET_LR_X = 0 # 3.5
OFFSET_LR_Y = 0 # 0

X_0 = 0.5
Y_0 = 0.5
Z_0 = 0
YAW_0 = 0

ALTITUDE_FILTER_LENGTH=5
ALTITUDE_FILTER_WEIGHTS=[0.06136,0.24477,0.38774,0.24477,0.06136]

ARRIVED_THRESHOLD=0.1*VELOCITY

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class Controller:
    """
    A controller class must be setup
    """
    def __init__(self, link_uri): 
        print("Start run")
        self.cf=Crazyflie(rw_cache='./cache')
        self.avoid_left = True
        self.state = SEARCH_STATE
        self.has_obstacle_ahead = False
        self.run()

    def set_initial_position(self, scf, x, y, z, yaw_deg):
        scf.cf.param.set_value('kalman.initialX', x)
        scf.cf.param.set_value('kalman.initialY', y)
        scf.cf.param.set_value('kalman.initialZ', z)
        yaw_radians = math.radians(yaw_deg)
        scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

    def start_communication(self):
            """This callback is called form the Crazyflie API when a Crazyflie
            has been connected and the TOCs have been downloaded."""
            # The definition of the logconfig can be made before connecting
            self._lg_conf = LogConfig(name='kalman', period_in_ms=100)
            # TODO: find the yaw log variable, and understand the difference between kalman and estimate
            self._lg_conf.add_variable('stateEstimate.x', 'float')
            self._lg_conf.add_variable('stateEstimate.y', 'float')
            self._lg_conf.add_variable('stateEstimate.z', 'float')
            self._lg_conf.add_variable('stateEstimate.yaw', 'float')

            # Adding the configuration cannot be done until a Crazyflie is
            # connected, since we need to check that the variables we
            # would like to log are in the TOC.
            try:
                # Set-up new callbacks for when the drone receives data
                self.cf.log.add_config(self._lg_conf)
                self._lg_conf.data_received_cb.add_callback(self.data_received_callback)
                self._lg_conf.error_cb.add_callback(self.error_received_callback)
                # Start the logging
                self._lg_conf.start()
            except KeyError as e:
                print('Could not start log configuration, {} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')

    def data_received_callback(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        self.pos = list(data.values())

    def error_received_callback(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def is_drone_in_final_area(self): 
        if self.pos[0] > OFFSET_LR_X and self.pos[0] < (OFFSET_LR_X+W_LR):
            return True
        else:
            return False

    # non blocking function that returns the position where the altitude estimate changed drastically.
        #mode changes the method used to detect the edge of the box
        #threshold is the height of the edge over which the edge should be detected in [m]       
    def boxEdgeDetection(self,mode,threshold):   
        if mode==0:
            #check if the needed variables exist and if not create them
            try: 
                self.oldAltitude
            except NameError: 
                self.oldAltitude=self.pos[2]    
            try: 
                self.filterArray
            except NameError:
                self.filterArray = []
            try: 
                self.differenceAccumulator
            except NameError:
                self.differenceAccumulator=0.0

            #insert new value into array used for filtering
            self.filterArray.append(self.pos)

            #remove the oldest datapoint from the filter array if filter longer than wanted
            if(len(self.filterArray>ALTITUDE_FILTER_LENGTH)):
                self.filterArray.pop(0)

            #prevent filtering if filter window is not full
            if(len(self.filterArray)==ALTITUDE_FILTER_LENGTH):
                self.filteredAltitude=0.0

                #calculate the weighted average(or finite gaussian filtered value depending on weights) of the altitude
                for arrayPosition in range(ALTITUDE_FILTER_LENGTH):
                    self.filteredAltitude=self.filteredAltitude+self.filterArray[arrayPosition][2]*ALTITUDE_FILTER_WEIGHTS[arrayPosition]

                #constantly sum the difference between each new altitude
                self.differenceAccumulator=self.differenceAccumulator+(self.filteredAltitude-self.oldAltitude)

                #small difference changes due to noise comming through the filter should equalize to 0,
                #but if a box is on the floor the decrease in detected altitude accumulates a big difference until the altutide controller has corrected it.
                if( abs(self.differenceAccumulator) >=threshold):
                    self.differenceAccumulator=0.0
                    return (True,self.filterArray[int(ALTITUDE_FILTER_LENGTH/2)+1])

        return (False,[])
    def run(self):
        """
        This function make the drone fly in TRAVEL mode. 
        """
        # Initialize the low-level drivers (don't list the debug drivers)
        with SyncCrazyflie(URI, cf=self.cf) as scf:
            # Set initial position 
            print("Set initial position")
            self.set_initial_position(scf, X_0, Y_0, Z_0, YAW_0)
            time.sleep(1)

            # we set the communication 
            self.start_communication()

            # We take off when the commander is created
            with MotionCommander(scf) as mc:
                with Multiranger(scf) as multiranger:
                    time.sleep(1)

                    while True: 
                        # In this while loop, we check for conditions that will change the travel state
                        # a. is there an obstacle ? 
                        if multiranger.front < THRESHOLD: 
                            print("Obstacle detected ", self.avoid_left)
                            if not self.has_obstacle_ahead:
                                self.has_obstacle_ahead = True

                            # there is an obstacle forward, so let's move to the left
                            # for a small amount of time
                            if (self.avoid_left and multiranger.left < THRESHOLD) \
                            or (not self.avoid_left and multiranger.right < THRESHOLD): 
                                print("Switching direction")
                                self.avoid_left = not self.avoid_left

                            if self.avoid_left: 
                                mc.left(0.2, velocity=VELOCITY)
                            else:
                                mc.right(0.2, velocity=VELOCITY)

                            # and then move again forward
                            mc.start_forward(velocity = VELOCITY)

                        elif self.has_obstacle_ahead:
                            # no more obstacles 
                            self.avoid_left = not self.avoid_left
                            self.has_obstacle_ahead = False

                        if self.state == TRAVEL_STATE:
                            # Start flying forward
                            mc.start_forward(velocity = VELOCITY)

                            # b. are we arrived in the landing area ? 
                            if self.is_drone_in_final_area():
                                # Then leave the travel mode
                                print("heheheh")
                                self.acquireNewRandomPosition=True
                                self.firstEdgeDetected=False
                                self.boxEdgePoints=[]
                                mc.stop()
                                return 
                        elif self.state == SEARCH_STATE:            #TODO: needs to be changed into a non blocking way to travel forward otherwise obstacle avoidance does not work
                            # a. genrate a point in the landing area
                            if(self.acquireNewRandomPosition==True):
                                self.targetPosition = np.random.rand(2) * np.array([W_LR , H_LR]) + [OFFSET_LR_X, OFFSET_LR_Y]
                                print("Next target:", self.targetPosition)
                                print("Current pos: ", self.pos)

                                # b. rotate towards the point
                                vector = self.targetPosition - self.pos[:2]
                                r = np.linalg.norm(vector) # propably not needed
                                delta = np.dot(vector, [math.cos(self.pos[-1]), math.sin(self.pos[-1])]) / r
                                delta = math.degrees(math.acos(delta))

                                print("Angle to rotate: ", delta)
                                if delta < 0:
                                    mc.turn_right(-delta)
                                else:
                                    mc.turn_left(delta)
                                self.acquireNewRandomPosition=False

                            # c. move forward     
                            #mc.forward(r)
                            mc.start_forward(velocity = VELOCITY)
                            if((np.linalg.norm(self.targetPosition - self.pos[:2])<ARRIVED_THRESHOLD) & self.firstEdgeDetected==False):
                                mc.stop()
                                mc.go_to(self.targetPosition[0],self.targetPosition[1])
                                self.acquireNewRandomPosition=True
                            [self.firstEdgeDetected,edgePoint] =self.boxEdgeDetection(0,0.1)
                            if(self.firstEdgeDetected==True):
                                mc.stop()
                                boxEdgePoints.append(edgePoint)
                                #self.state= LANDING_STATE





if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    Controller(URI)

