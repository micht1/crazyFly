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

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            with Multiranger(scf) as multiranger:
                time.sleep(1)

                # Start flying forward
                mc.start_forward(velocity = VELOCITY)
                while True: 
                    if multiranger.front < THRESHOLD: 
                        # there is an obstacle forward, so let's move to the left
                        # for a small amount of time
                        mc.start_left(velocity=VELOCITY)
                        time.sleep(0.2)
                        # and then move again forward
                        mc.start_forward(velocity = VELOCITY)

                # And we can stop
                mc.stop()

                # We land when the MotionCommander goes out of scope
