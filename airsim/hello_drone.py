"""
For connecting to the AirSim drone environment and testing API functionality

USAGE: python SCRIPT_NAME AIRSIM_SERVER_IP DRONE_ID
"""

import os
import time
import tempfile
import pprint
import sys
from AirSimClient import *

# read argv
ip = sys.argv[1]
port = 41451 + int(sys.argv[2])

print port

# connect to the AirSim simulator
client = MultirotorClient(ip, port)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

client.takeoff()

#client.wait_key('press anykey')

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

client.moveToPosition(-20, 100, -30, 5)

client.hover()


time.sleep(5)

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))


print("reset to original state")

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)


