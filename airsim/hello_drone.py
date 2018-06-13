"""
For connecting to the AirSim drone environment and testing API functionality
"""

import os
import tempfile
import pprint

from AirSimClient import *


# connect to the AirSim simulator
client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

client.takeoff()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

#client.moveToPosition(-10, 10, -10, 5)

client.hover()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)

