"""
For connecting to the AirSim drone environment and testing API functionality
"""

import os
import sys
import tempfile
import pprint
import math

from AirSimClient import *

g_drone_id = 0
g_image_id = 1
g_image_folder = 'G:\\my_github\\AirSim\\Unreal\\Environments\\Blocks\\my_img\\'

def mylog(msg):
    print('[drone_{}] {}'.format(g_drone_id, msg))

def mylog_screen():
    global g_drone_id, g_image_id, g_image_folder
    # get camera images from the car
    responses = client.simGetImages([
        ImageRequest(0, AirSimImageType.DepthVis, pixels_as_float=True),  # center front, depth visualiztion image
        ImageRequest(0, AirSimImageType.Scene), # center front, scene vision image in png format
    ])
    mylog('Retrieved images: %d' % len(responses))

    tmp_dir = g_image_folder + 'drone_{}\\'.format(g_drone_id)
    mylog ("Saving images to %s" % tmp_dir)
    if not os.path.isdir(tmp_dir):
        try:
            os.makedirs(tmp_dir)
        except OSError:
            raise
    filename_prefix = tmp_dir + "{}-".format(g_image_id)
    g_image_id += 1
    for idx, response in enumerate(responses):
        filename = filename_prefix + str(idx)
        if response.pixels_as_float:
            mylog("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            AirSimClientBase.write_pfm(os.path.normpath(filename + '.pfm'), AirSimClientBase.getPfmArray(response))
        elif response.compress: #png format
            mylog("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            AirSimClientBase.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else: #uncompressed array
            mylog("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
            img_rgba = img1d.reshape(response.height, response.width, 4) #reshape array to 4 channel image array H X W X 4
            img_rgba = np.flipud(img_rgba) #original image is fliped vertically
            img_rgba[:,:,1:2] = 100 #just for fun add little bit of green in all pixels
            AirSimClientBase.write_png(os.path.normpath(filename + '.greener.png'), img_rgba) #write to png



def degreesToRadians(degrees):
    return math.pi * degrees / 180.0

def GeodeticToNedFast(geo, home):
    d_lat = geo.latitude - home.latitude;
    d_lon = geo.longitude - home.longitude;
    d_alt = home.altitude - geo.altitude;
    EARTH_RADIUS = 6378137.0
    x = degreesToRadians(d_lat) * EARTH_RADIUS
    y = degreesToRadians(d_lon) * EARTH_RADIUS * math.cos(degreesToRadians(geo.latitude))
    return Vector3r(x, y, d_alt)

def nedToGeodeticFast(local, home):
    d_lat = local.x_val / EARTH_RADIUS;
    d_lon = local.y_val / (EARTH_RADIUS * math.cos(degreesToRadians(home.latitude)));
    latitude = home.latitude + radiansToDegrees(d_lat);
    longitude = home.longitude + radiansToDegrees(d_lon);
    altitude = home.altitude - local.z_val;
    return GeoPoint(latitude,longitude,altitude)

def getCalibratedPos(pos, ref):
    return Vector3r(pos.x_val - ref.x_val, pos.y_val - ref.y_val, pos.z_val - ref.z_val)

def getCalibratedPosFromGps(gps, home, ref):
    pos = GeodeticToNedFast(gps, home)
    return Vector3r(pos.x_val - ref.x_val, pos.y_val - ref.y_val, pos.z_val - ref.z_val)


argc = len(sys.argv)
if argc < 1:
    mylog('usage: python3 hello_drone2.py DRONE_ID')
    exit(0)
g_drone_id = int(sys.argv[1])
# connect to the AirSim simulator
ip = "127.0.0.1"
port = 41451 + g_drone_id
client = MultirotorClient(ip, port)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)


landed = client.getLandedState()
if landed == LandedState.Landed:
    print("taking off...")
    client.takeoff()
else:
    print("already flying...")
    client.hover()




home_gps_location = client.getGpsLocation()
pos = GeodeticToNedFast(home_gps_location, home_gps_location)
print('gps={}, pos={}'.format(home_gps_location, pos))

#client.simSetPose(
#        Pose(Vector3r(0, -20, 0), AirSimClientBase.toQuaternion(0, 0, 0)), 
#        True) 

print('simGetObjectPose={}'.format(client.simGetObjectPose('myOrigin')))
print('simGetObjectPose={}'.format(client.simGetObjectPose('FP1')))
print('simGetObjectPose={}'.format(client.simGetObjectPose('FP2')))
print('simGetObjectPose={}'.format(client.simGetObjectPose('FP3')))
print('simGetObjectPose={}'.format(client.simGetObjectPose('FP4')))

state = client.getMultirotorState()
mylog("state: %s" % pprint.pformat(state))

cur_pos = client.getPosition()
origin_pos = client.simGetObjectPose('myOrigin').position
print('getCalibratedPos={}'.format(getCalibratedPos(cur_pos, origin_pos)))


while True:
    scale = 5
    pressed_key = AirSimClientBase.wait_key(
        'Press AWSD,R(up)F(down) key to move vehicle {}m at {}m/s.\n'.format(scale, scale)
        + 'Press P to take images.\n'
        + 'Press B key to reset to original state.\n'
        + 'Press Q key to release API control.\n'
    )
    pressed_key = pressed_key.decode('utf-8')
    mylog("pressed_key={}".format(pressed_key))
    if pressed_key == 'L' or pressed_key == 'l':
        print('{}'.format(client.getPosition()))
        client.simSetPose(
                Pose(Vector3r(20, 20, -30), AirSimClientBase.toQuaternion(0, 0, 0)), 
                True)
    if pressed_key == 'P' or pressed_key == 'p':
        mylog_screen()
    elif pressed_key == 'Q' or pressed_key == 'q':
        client.enableApiControl(False)
        break
    elif pressed_key == 'B' or pressed_key == 'b':
        client.reset()
    else:
        move_x, move_y, move_z, speed = 0, 0, 0, 1
        speed = speed*scale
        if pressed_key == 'A' or pressed_key == 'a':
            move_y = -1*scale
        elif pressed_key == 'D' or pressed_key == 'd':
            move_y = +1*scale
        elif pressed_key == 'W' or pressed_key == 'w':
            move_x = +1*scale
        elif pressed_key == 'S' or pressed_key == 's':
            move_x = -1*scale
        elif pressed_key == 'R' or pressed_key == 'r':
            move_z = -1*scale
        elif pressed_key == 'F' or pressed_key == 'f':
            move_z = +1*scale
        cur_pos_vector3r = client.getPosition()
        cur_pos = (cur_pos_vector3r.x_val, cur_pos_vector3r.y_val, cur_pos_vector3r.z_val)
        next_pos = (cur_pos[0] + move_x, cur_pos[1] + move_y, cur_pos[2] + move_z)
        mylog("try to move: {} -> {}".format(cur_pos, next_pos))
        
        # is collision?
        collision_info = client.getCollisionInfo()
        if collision_info.has_collided:
            mylog("Collision at pos %s, normal %s, impact pt %s, penetration %f, name %s, obj id %d" % (
                pprint.pformat(collision_info.position), 
                pprint.pformat(collision_info.normal), 
                pprint.pformat(collision_info.impact_point), 
                collision_info.penetration_depth, collision_info.object_name, collision_info.object_id))
            break
        rc = client.moveToPosition(next_pos[0], next_pos[1], next_pos[2], speed)
        mylog("rc: {}".format(rc))

        cur_pos_vector3r = client.getPosition()
        print('cur_pos_vector3r={}'.format(cur_pos_vector3r))
        print('getCalibratedPos={}'.format(getCalibratedPos(cur_pos_vector3r, origin_pos)))
        #state = client.getMultirotorState()
        #mylog("state: %s" % pprint.pformat(state))


