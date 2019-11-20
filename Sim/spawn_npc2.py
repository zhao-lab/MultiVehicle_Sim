"""
Script for spawning random vehicles.
The spawn points are sampled using a distribution from the list of available spawn points.
To simulate naturalistic traffic.
[Spawning all vehicles sequentially, checking for collision at each spawn instant.]
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

    def __init__(self, world):
        # ----------NGSim1-------------------------------------------------
        # self.ROI_in=[[-1720,-1956.5],[-1657, -1928],[-1660, -1838],[-1738,-1887]]
        # self.ROI_ou=[[-1734,-1986.5],[-1631, -1927],[-1631,-1754.5],[-1785,-1870]]

        # --------Carla Map Town 04----------------------------
        self.ROI_in = [[281.13,224.60],[281.13, 269.35],[232.55, 269.35],[232.55, 224.60]]
        self.ROI_ou = [[300.13,204.60],[301.13, 289.35],[212.55, 289.35],[212.55, 204.60]]
        self.world = world
        self.map = world.get_map()
        self.spawn_points = []
        self.last_frame = 0
        self.blueprints = world.get_blueprint_library().filter('vehicle.toyota.*')
        self.actors=[]

    def sign(self,a,b,c):
        return (a[0]-c[0])*(b[1]-c[1])-(b[0]-c[0])*(a[1]-c[1])

    def ptinT(self,x,a,b,c):
        d1 = self.sign(x,a,b)
        d2 = self.sign(x,b,c)
        d3 = self.sign(x,c,a)
        has_neg = (d1<0) or (d2<0) or (d3<0)
        has_pos = (d1>0) or (d2>0) or (d3>0)
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
        # spawn_points=self.map.get_spawn_points()    #   Querying spawnpoints
        # waypoints=self.map.generate_waypoints(5.0)   # Generating waypoints at a resolution of 5m
    
        # self.spawn_points=[]
        # for i,w in enumerate(waypoints):
        #     x=[w.transform.location.x,-w.transform.location.y]
        #     # if i==0:
        #     #     wpts=np.array(x).reshape(1,-1)
        #     # else:
        #     #     wpts=np.vstack((wpts,np.array(x).reshape(1,-1)))

        #     if(self.inROI(x,self.ROI_ou) and not(self.inROI(x,self.ROI_in))):
        #         spawn_point=w.transform
        #         spawn_point.location.z=2
        #         self.spawn_points.append(spawn_point)


        # -------------------------Map Town04, Unsignalied 4way intersection
        self.spawn_points=self.map.get_spawn_points()[:4]
        spawnx=[255.257156, 258.637909, 294.50, 222.266479 ]
        spawny=[-282.577789, -209.931717, -250.234390, -245.930328]
        spawnyaw=[90.172272, -88.316437, -179.65, -0.500977]
        for i in range(4):
            self.spawn_points[i].location.x=spawnx[i]
            self.spawn_points[i].location.y=spawny[i]
            self.spawn_points[i].rotation.yaw=spawnyaw[i]
            # self.spawn_points.append(spawn_point)
        
        print('Number of spawn points in the hanger: ',len(self.spawn_points))

    def spawn_vehicles(self, frame, df=100):
        """
        Function to spawn vehicles at the list of spawn points.
        """
        if (frame - self.last_frame < df):
            return 
        else:
            # spts=random.choice(len(self.spawnpoints))
            for i in range(len(self.spawn_points)):
                # Randomly choosing spawn points, (Naturalisitic Traffic simulation)
                spawn_point = self.spawn_points[i]     
                blueprint = random.choice(self.blueprints)
                actor = self.world.try_spawn_actor(blueprint, spawn_point)
                if(actor == None):
                    continue
                actor.set_autopilot()   # Setting the vehicle control in autpolit mode
                self.actors.append(actor)
            self.last_frame = frame

    def check_VState(self):
        """
        To remove the vehicles (kill the actors) that have moved out of the region of interest...
        """
        for actor in self.actors:
            lc = actor.get_transform().location
            if not(self.inROI([lc.x, -lc.y], self.ROI_ou)):
                actor.destroy()
                self.actors.remove(actor)


    def destroy_actors(self):
        """
        To destroy all the actors in the simulation
        """
        for actor in self.actors:
            actor.destroy()


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
        '--fps',
        default=30,
        type=int,
        help='The simulation fps')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(3.0)
        world = client.get_world()

        # Instantiating an object of the spawner class
        garage=spawner(world)   
        garage.spawn_hangar()

        # Configuring the simulation to run at a constant fps and in sycnhronous mode.
        fps = args.fps
        settings = world.get_settings()
        settings.sycnhronous_mode = True
        settings.fixed_delta_seconds = 1.0/fps
        world.apply_settings(settings)
        print(dir(world.tick()))
        while(True):
            try:
                frame = world.tick()
            except RuntimeError:
                continue
            # print(frame)
            # Spawning Vehicles based on random distribution
            garage.spawn_vehicles(frame)

            # Checking and removing vehicles that have moved outside ROI
            garage.check_VState()
    finally:
        print("Entered Finally, destroying {} vehicles".format(len(garage.actors)))
        garage.destroy_actors()   

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
