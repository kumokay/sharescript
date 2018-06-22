"""
For connecting to the AirSim drone environment and testing API functionality
"""

import os
import sys
import tempfile
import pprint

from AirSimClient import *

g_drone_id = 0
g_image_id = 1
g_image_folder = 'G:\\my_github\\AirSim\\Unreal\\Environments\\Blocks\\my_img\\'

def mylog(msg, tag = 'all'):
    print('[{}] {}'.format(tag, msg))

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

class DroneControl:
    drone_id = 0
    client = None
    speed = 20  # m/s
    min_height = -35  # idk why height is a negtive number...
    max_height = 0.0
    home_origin = None
    ref_origin = None
    home_gps = None


    @classmethod
    def degreesToRadians(degrees):
        return math.pi * degrees / 180.0

    @classmethod
    def GeodeticToNedFast(geo, home):
        d_lat = geo.latitude - home.latitude;
        d_lon = geo.longitude - home.longitude;
        d_alt = home.altitude - geo.altitude;
        EARTH_RADIUS = 6378137.0
        x = degreesToRadians(d_lat) * EARTH_RADIUS
        y = degreesToRadians(d_lon) * EARTH_RADIUS * math.cos(degreesToRadians(geo.latitude))
        return Vector3r(x, y, d_alt)

    @classmethod
    def nedToGeodeticFast(local, home):
        d_lat = local.x_val / EARTH_RADIUS;
        d_lon = local.y_val / (EARTH_RADIUS * math.cos(degreesToRadians(home.latitude)));
        latitude = home.latitude + radiansToDegrees(d_lat);
        longitude = home.longitude + radiansToDegrees(d_lon);
        altitude = home.altitude - local.z_val;
        return GeoPoint(latitude,longitude,altitude)

    def getCalibratedPos(self):
        pos = self.client.getPosition()
        return Vector3r(
                pos.x_val - self.ref_origin.x_val, 
                pos.y_val - self.ref_origin.y_val, 
                pos.z_val - self.ref_origin.z_val)
    def convertCalibrated2NonCalibrated(self, pos):
        return Vector3r(
                pos.x_val - self.home_origin.x_val, 
                pos.y_val - self.home_origin.y_val, 
                pos.z_val - self.home_origin.z_val)

    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.log('create drone contrller for drone_{}'.format(self.drone_id))
    def log(self, msg):
        mylog(msg, tag='drone_{}'.format(self.drone_id))
    
    def connect(self):
        # connect to the AirSim simulator
        self.client = MultirotorClient('127.0.0.1', 41451 + self.drone_id) # ip, port
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        state = self.client.getMultirotorState()
        self.log('state: {}'.format(state))
        self.ref_origin = self.client.simGetObjectPose('myOrigin').position
        self.home_origin = self.getCalibratedPos() 
        self.home_gps = self.client.getGpsLocation()
        self.log('ref_origin: {}'.format(self.ref_origin))
        self.log('home_origin: {}'.format(self.home_origin))
        self.log('home_gps: {}'.format(self.home_gps))
    def takeoff(self):
        # self.client.takeoff()
        self._moveToPosNonCali(Vector3r(0, 0, self.min_height))
    def moveToPos(self, pos):
        pos = self.convertCalibrated2NonCalibrated(pos)
        self._moveToPosNonCali(pos)
    def _moveToPosNonCali(self, pos):
        cur_pos = self.client.getPosition()
        distance = math.sqrt((pos.x_val - cur_pos.x_val)**2 + (pos.y_val - cur_pos.y_val)**2 + (pos.z_val - cur_pos.z_val)**2)
        speed = self.speed if self.speed <= distance else distance
        # move to pos
        rc = self.client.moveToPosition(pos.x_val, pos.y_val, pos.z_val, speed)
        self.log("rc: {}".format(rc))
        self.log("pos: {}".format(self.getCalibratedPos()))
        self.log("gps: {}".format(self.getGpsLocation()))
    def moveToDirction(self, direction):
        cur_pos = self.client.getPosition()
        dst_pos = Vector3r(cur_pos.x_val+direction.x_val, cur_pos.y_val+direction.y_val, cur_pos.z_val+direction.z_val)
        self._moveToPosNonCali(pos)
    def moveToGps(self, gps):
        noncali_pos = GeodeticToNedFast(gps, self.home_gps)
        self._moveToPosNonCali(noncali_pos)
    def disconnect(self):
        self.client.enableApiControl(False)
    def goHome(self):
        self.client.reset()

class DroneType:
    AccessPoint = 0
    Station = 1

n_max_drones = 4
drone_list = [  # type, location, controller
    [DroneType.AccessPoint, Vector3r(50, 0, -30),  None],
    [DroneType.AccessPoint, Vector3r(100, 0, -30), None],
    [DroneType.AccessPoint, Vector3r(0, 50, -30),  None],
    [DroneType.AccessPoint, Vector3r(0, 100, -30), None],
    #[DroneType.Station,   Vector3r(30, 30, 20), None],
    #[DroneType.Station,   Vector3r(60, 60, 20), None],
]
n_drones = len(drone_list)
assert(n_drones <= n_max_drones)
for i in range(n_drones):
    drone_list[i][2] = DroneControl(i)

for i in range(n_drones):
    drone = drone_list[i][2]
    drone.connect()
    drone.takeoff()

for i in range(n_drones):
    drone = drone_list[i][2]
    drone.moveToPos(drone_list[i][1])



    

