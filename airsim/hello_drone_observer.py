"""
Observe environment and setup mininet

USAGE: python SCRIPT_NAME AIRSIM_SERVER_IP DRONE_ID
"""

import os
import time
import tempfile
import pprint
import sys
import logging
import msgpackrpc
from AirSimClient import *

import lib_geo

def rpc_call(rpcClient, *args): 
    cmd = args[0]
    logging.info('rpc_call: {}'.format(cmd))
    result = rpcClient.call(*args)
    logging.info('result: {}'.format(result))

######################## MAIN ########################

logging.basicConfig(level=logging.DEBUG)

# pre-defined object tags in airsim map
arisim_tag_AP = ['AP1', 'AP2', 'AP3']
arisim_tag_Drone = ['Drone0', 'Drone1']
airsim_max_drones = 2

# read argv
ip = '172.17.20.12'
drone_id = 0
if len(sys.argv) > 1:
    ip = sys.argv[1]
    drone_id = int(sys.argv[2])  # start from 0
port = 41451 + drone_id
if drone_id >= airsim_max_drones:
    logging.error('error drone id: {}; airsim_max_drones={}'.format(drone_id, airsim_max_drones))
    exit(0)

logging.info('drone_{}: connect to airsim server: {}:{}'.format(drone_id, ip, port))
client = MultirotorClient(ip, port)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

logging.info('drone_{}: prepare to takeoff'.format(drone_id))
client.takeoff()

logging.info('get ref points locations')
ref_pos = client.simGetObjectPose('AP1').position


logging.info('get AP locations')
dict_AP = {}
for tag in arisim_tag_AP:
    pos = lib_geo.getCalibratedPos(client.simGetObjectPose(tag).position, ref_pos)
    logging.info('simGetObjectPose: {} at {}'.format(tag, pos))
    dict_AP[tag] = pos.to_string()

logging.info('get drone locations')
dict_Drone = {}
for tag in arisim_tag_Drone:
    pos = lib_geo.getCalibratedPos(client.simGetObjectPose(tag).position, ref_pos)
    logging.info('simGetObjectPose: {} at {}'.format(tag, pos))
    dict_Drone[tag] = pos.to_string()

logging.info('get client location')
home_gps_location = client.getGpsLocation()
client_pos = lib_geo.getCalibratedPosFromGps(home_gps_location, home_gps_location, ref_pos)
# AP1 is the original point
logging.info('getCalibratedPosFromGps: {} at {}'.format('client', client_pos))


ip = '172.17.20.12'
port = 18800
rpcClient = msgpackrpc.Client(msgpackrpc.Address(ip, port))
rpc_call(rpcClient, 'create_empty_net')
rpc_call(rpcClient, 'add_nodes', dict_AP, dict_Drone)
rpc_call(rpcClient, 'start_network')
tag = 'Drone1'
pos = lib_geo.getCalibratedPos(client.simGetObjectPose(tag).position, ref_pos)
logging.info('simGetObjectPose: {} at {}'.format(tag, pos))
from_pos = pos.to_string()
to_pos = (pos.add(20,20,20)).to_string()
rpc_call(rpcClient, 'start_mobility', tag, from_pos, to_pos, 2)
rpc_call(rpcClient, 'start_cil')
