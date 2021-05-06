"""
TRAVEL MODE
-----------

Goes forward until the robot meets an obstacle in front.
Then it moves to one orthogonal direction.
Then it start traveling forward again.

"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/70/2M'
THRESHOLD = 20 
VELOCITY = 0.5

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class Controller:
    """
    A controller class must be setup
    """
    def __init__(self): 
        self.cf=Crazyflie(rw_cache='./cache')
        # add a callback for when the drone is connected
        self.cf.connected.add_callback(self.connected)


    def connected(self, link_uri):
            """This callback is called form the Crazyflie API when a Crazyflie
            has been connected and the TOCs have been downloaded."""
            print('Connected to %s' % link_uri)

            # The definition of the logconfig can be made before connecting
            self._lg_conf = LogConfig(name='kalman', period_in_ms=100)
            self._lg_conf.add_variable('kalman.stateX', 'float')
            self._lg_conf.add_variable('kalman.stateY', 'float')
            self._lg_conf.add_variable('kalman.stateZ', 'float')

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

            # Start a timer to disconnect in 10s
            # (eventually)
            t = Timer(5, self.cf.close_link)
            t.start()

            # And start the running code
            self.run()

    def data_received_callback(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()

    def error_received_callback(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def is_drone_in_final_area(self, pos): 
        return False

    def run(self):
        """
        This function make the drone fly in TRAVEL mode. 
        """
        # Initialize the low-level drivers (don't list the debug drivers)
        with SyncCrazyflie(URI, cf=cf)) as scf:
            # We take off when the commander is created
            with MotionCommander(scf) as mc:
                with Multiranger(scf) as multiranger:
                    time.sleep(1)

                    # Start flying forward
                    mc.start_forward(velocity = VELOCITY)
                    while True: 
                        # In this while loop, we check for conditions that will change the travel state
                        # a. is there an obstacle ? 
                        if multiranger.front < THRESHOLD: 
                            # there is an obstacle forward, so let's move to the left
                            # for a small amount of time
                            mc.start_left(velocity=VELOCITY)
                            time.sleep(0.2)
                            # and then move again forward
                            mc.start_forward(velocity = VELOCITY)

                        # b. are we arrived in the landing area ? 
                        drone_pos = None# TODO
                        if self.is_drone_in_final_area(drone_pos):
                            # Then leave the travel mode
                            mc.stop()

                        
                    # And we can stop
                    mc.stop()

                    # We land when the MotionCommander goes out of scope



if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    Controller()

