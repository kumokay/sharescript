"""
Observe environment and setup mininet

USAGE: python SCRIPT_NAME AIRSIM_SERVER_IP DRONE_ID
"""


import sys
import msgpackrpc
import logging
import lib_geo
from AirSimClient import Vector3r

# import mininet-wifi path
sys.path.append('/home/kumokay/github/mininet-wifi')
from mininet.node import RemoteController, Node, Docker
from mininet.log import setLogLevel, info
from mininet_wifi.wifi.node import OVSKernelAP, Station, DockerStation
from mininet_wifi.wifi.cli import CLI_wifi
from mininet_wifi.wifi.net import Mininet_wifi


class MininetServer(object):
    # expect sequence:
    #  -> create_empty_net
    #  -> add_nodes
    #  -> start_network
    #  -> start_mobility
    #  -> start_mobility
    #  -> start_mobility
    #  ...
    #  -> stop_network

    # variables
    c1 = None
    net = None
    APs = {}
    Stations = {}
    Hosts = {}

    def create_empty_net(self):
        logging.info('setup_empty_net')
        self.net = Mininet_wifi(accessPoint=OVSKernelAP)
        self.net.propagationModel(model="logDistance", exp=3)
        self.c1 = RemoteController('c1', ip='172.17.20.12', port=6633)
        self.net.addController(self.c1)
        return True

    def add_nodes(self, dict_AP, dict_Station):
        logging.info('add_nodes')
        dimage_name='kumokay/ubuntu_wifi:v2'

        logging.debug('add APs: {}'.format(dict_AP))
        for name in dict_AP:
            pos = dict_AP[name]
            self.APs[name] = self.net.addAccessPoint(
                    name,
                    ssid='new-ssid',
                    mode='g',
                    channel='1',
                    position=pos)

        logging.debug('add Stations: {}'.format(dict_Station))
        ip_lastbyte = 10
        for name in dict_Station:
            pos = dict_Station[name]
            self.Stations[name] = self.net.addStation(
                    name,
                    cls=DockerStation,
                    dimage=dimage_name,
                    mac='00:00:00:00:00:{:x}'.format(ip_lastbyte),
                    ip='10.0.0.{}/8'.format(ip_lastbyte),
                    position=pos)
            ip_lastbyte += 1

        logging.debug('configure wifi nodes')
        self.net.configureWifiNodes()

        host_per_ap = 1
        logging.info('add Hosts: by default, each AP connected to 1 hosts')
        ip_lastbyte = 100
        for ap_name in self.APs:
            for i in range(0,host_per_ap):
                host_name = '{}-h{}'.format(ap_name, i)
                self.Hosts[host_name] = self.net.addHost(
                        host_name,
                        cls=Docker,
                        dimage=dimage_name,
                        mac='00:00:00:00:00:{:x}'.format(ip_lastbyte),
                        ip='10.0.0.{}/8'.format(ip_lastbyte))
                ip_lastbyte += 1
                self.net.addLink(self.APs[ap_name], self.Hosts[host_name])
        return True

    def start_network(self):
        logging.info('start_network')
        self.net.build()
        self.c1.start()
        for ap_name in self.APs:
            self.APs[ap_name].start([self.c1])
        return True

    def stop_network(self):
        logging.info('stop_network')
        self.net.stop()
        return True

    def start_mobility(self, station_name, from_pos, to_pos, velocity):
        logging.info('start_mobility')
        logging.debug(self.Stations)
        if station_name not in self.Stations:
            logging.warning('no such station: {}'.format(station_name))
            return False
        logging.info('{} -> {}'.format(from_pos, to_pos))
        v_from = Vector3r().init_from_string(from_pos)
        v_to = Vector3r().init_from_string(to_pos)
        logging.info('{} -> {}'.format(v_from, v_to))
        distance = lib_geo.getDistance(v_to, v_from)
        logging.info('{}'.format(distance))
        delta_t = int(distance / velocity)
        logging.info('move {} from {} to {}, velocity={}, dist={}, delta_t={}'.format(
                station_name,
                from_pos,
                to_pos,
                velocity,
                distance,
                delta_t))
        station = self.Stations[station_name]
        self.net.startMobility(time=0, repetitions=0)
        self.net.mobility(station, 'start', time=1, position=from_pos)
        self.net.mobility(station, 'stop', time=delta_t+1, position=to_pos)
        self.net.stopMobility(time=delta_t+2)
        return True

    def start_cil(self):
        CLI_wifi(self.net)
        return True

######################## MAIN ########################

logging.basicConfig(level=logging.DEBUG)

ip = '172.17.20.12'
port = 18800
logging.info('starting msgpackrpc at {}:{}'.format(ip, port))
server = msgpackrpc.Server(MininetServer())
server.listen(msgpackrpc.Address(ip, port))
server.start()
