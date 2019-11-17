"""
Script for spawning random vehicles around the ROI (Region of Interest).
To simulate naturalistic traffic.
[Ref: spawn_npc.py in Carla/PythonAPI/examples]

Author: Ashish Roongta
SafeAI lab, Carnegie Mellon University
Copyright @ SafeAI lab-Carengie Mellon University
"""

"""Spawn NPCs into the simulation"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import random
import numpy as np
import time

class spawner:

    def __init__(self, map1):
        # self.ROI_in=[[-1720,-1956.5],[-1657, -1928],[-1660, -1838],[-1738,-1887]]
        # self.ROI_ou=[[-1734,-1986.5],[-1631, -1927],[-1631,-1754.5],[-1785,-1870]]
        self.ROI_in=[[281.13,224.60],[281.13, 269.35],[232.55, 269.35],[232.55, 224.60]]
        self.ROI_ou=[[311.13,204.60],[311.13, 289.35],[202.55, 289.35],[202.55, 204.60]]
        self.map=map1
        self.spawn_points=[]

    def sign(self,a,b,c):
        return (a[0]-c[0])*(b[1]-c[1])-(b[0]-c[0])*(a[1]-c[1])

    def ptinT(self,x,a,b,c):
        d1=self.sign(x,a,b)
        d2=self.sign(x,b,c)
        d3=self.sign(x,c,a)
        has_neg=(d1<0) or (d2<0) or (d3<0)
        has_pos=(d1>0) or (d2>0) or (d3>0)
        return not(has_neg and has_pos)
    
    def inROI(self,x,ROI):
        """
        Function to check if the vehicle is in the region of interest.
        Returns True/ False.
        Enter the ROI (x,-y) coordinates in cyclic order.
        """

        # ROI=self.ROI
        # Checking if the current vehicle position lies within the region of interest
        d1=self.sign(x,ROI[0],ROI[1])
        d2=self.sign(x,ROI[1],ROI[2])
        d3=self.sign(x,ROI[2],ROI[3])
        d4=self.sign(x,ROI[3],ROI[0])

        has_neg=(d1<0) or (d2<0) or (d3<0) or (d4<0)
        has_pos=(d1>0) or (d2>0) or (d3>0) or (d4>0)
        return not(has_neg and has_pos)

    def spawn_hangar(self):
        """
        Function to generate list of possible spawn points
        """
        # NGSim1 - lankerShim Boulevard.....................................
        # # delta=20    # Width of surrounding region of the ROI to query spawn points
        spawn_points=self.map.get_spawn_points()    #   Querying spawnpoints
        waypoints=self.map.generate_waypoints(5.0)   # Generating waypoints at a resolution of 5m
    
        self.spawn_points=[]
        waypoints=self.map.generate_waypoints(7.0)
        for i,w in enumerate(waypoints):
            x=[w.transform.location.x,-w.transform.location.y]
            # if i==0:
            #     wpts=np.array(x).reshape(1,-1)
            # else:
            #     wpts=np.vstack((wpts,np.array(x).reshape(1,-1)))

            if(self.inROI(x,self.ROI_ou) and not(self.inROI(x,self.ROI_in))):
                spawn_point=w.transform
                spawn_point.location.z=2
                self.spawn_points.append(spawn_point)


        # -------------------------Map Town04, Unsignalied 4way intersection
        
        # self.spawn_points=self.map.get_spawn_points()[:4]
        # spawnx=[255.257156, 258.637909, 294.50, 222.266479 ]
        # spawny=[-282.577789, -209.931717, -250.234390, -245.930328]
        # spawnyaw=[90.172272, -88.316437, -179.65, -0.500977]
        # for i in range(4):
        #     self.spawn_points[i].location.x=spawnx[i]
        #     self.spawn_points[i].location.y=spawny[i]
        #     self.spawn_points[i].rotation.yaw=spawnyaw[i]
        #     # self.spawn_points.append(spawn_point)
        
        print('Number of spawn points in the hanger: ',len(self.spawn_points))



def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=50,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    spawnNow=True
    try:
        
        world = client.get_world()
        garage=spawner(world.get_map())
        blueprints = world.get_blueprint_library().filter('vehicle.toyota.*')

        # spawn_points = world.get_map().get_spawn_points()
        garage.spawn_hangar()
        spawn_points=garage.spawn_points

        number_of_spawn_points = len(spawn_points)
        # print('num of waypoints',number_of_spawn_points)

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        while(True):
            world.wait_for_tick()
            # --------------
            # Spawn vehicles
            # --------------
            batch=[]
            if spawnNow:
                for n,waypt in enumerate(spawn_points):
                    transform=waypt
                    blueprint=random.choice(blueprints)
                    batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
                client.apply_batch_sync(batch)
                spawnNow=False

            vehicles_list=world.get_actors().filter('vehicle.*')
            if(len(vehicles_list)<=0):
                spawnNow=True

            for car in vehicles_list:
                lc=car.get_transform().location
                if not(garage.inROI([lc.x,-lc.y],garage.ROI_ou)):
                    car.destroy()

    finally:

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controler, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
