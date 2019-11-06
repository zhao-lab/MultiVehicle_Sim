#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

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
        self.ROI_in=[[-1720,-1956.5],[-1657, -1928],[-1660, -1838],[-1738,-1887]]
        self.ROI_ou=[[-1734,-1986.5],[-1631, -1927],[-1631,-1754.5],[-1785,-1870]]
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
        # delta=20    # Width of surrounding region of the ROI to query spawn points
        # spawn_points=self.map.get_spawn_points()    #   Querying spawnpoints
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
            if(len(vehicles_list)<=1):
                spawnNow=True

            for car in vehicles_list:
                lc=car.get_transform().location
                if not(garage.inROI([lc.x,-lc.y],garage.ROI_ou)):
                    car.destroy()

            # client.apply_batch_sync([carla.command.SpawnActor])
        # batch = []
        # for n, waypnt in enumerate(spawn_points):
        #     transform=waypnt   #    .transform
        #     if n >= args.number_of_vehicles:
        #         break
        #     blueprint = random.choice(blueprints)
        #     if blueprint.has_attribute('color'):
        #         color = random.choice(blueprint.get_attribute('color').recommended_values)
        #         blueprint.set_attribute('color', color)
        #     if blueprint.has_attribute('driver_id'):
        #         driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        #         blueprint.set_attribute('driver_id', driver_id)
        #     blueprint.set_attribute('role_name', 'autopilot')
        #     batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

        # for response in client.apply_batch_sync(batch):
        #     if response.error:
        #         logging.error(response.error)
        #     else:
        #         vehicles_list.append(response.actor_id)

        # # -------------
        # # Spawn Walkers
        # # -------------
        # # 1. take all the random locations to spawn
        # spawn_points = []
        # for i in range(args.number_of_walkers):
        #     spawn_point = carla.Transform()
        #     loc = world.get_random_location_from_navigation()
        #     if (loc != None):
        #         spawn_point.location = loc
        #         spawn_points.append(spawn_point)
        # # 2. we spawn the walker object
        # batch = []
        # for spawn_point in spawn_points:
        #     walker_bp = random.choice(blueprintsWalkers)
        #     # set as not invencible
        #     if walker_bp.has_attribute('is_invincible'):
        #         walker_bp.set_attribute('is_invincible', 'false')
        #     batch.append(SpawnActor(walker_bp, spawn_point))
        # results = client.apply_batch_sync(batch, True)
        # for i in range(len(results)):
        #     if results[i].error:
        #         logging.error(results[i].error)
        #     else:
        #         walkers_list.append({"id": results[i].actor_id})
        # # 3. we spawn the walker controller
        # batch = []
        # walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        # for i in range(len(walkers_list)):
        #     batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        # results = client.apply_batch_sync(batch, True)
        # for i in range(len(results)):
        #     if results[i].error:
        #         logging.error(results[i].error)
        #     else:
        #         walkers_list[i]["con"] = results[i].actor_id
        # # 4. we put altogether the walkers and controllers id to get the objects from their id
        # for i in range(len(walkers_list)):
        #     all_id.append(walkers_list[i]["con"])
        #     all_id.append(walkers_list[i]["id"])
        # all_actors = world.get_actors(all_id)

        # # wait for a tick to ensure client receives the last transform of the walkers we have just created
        # world.wait_for_tick()

        # # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # for i in range(0, len(all_id), 2):
        #     # start walker
        #     all_actors[i].start()
        #     # set walk to random point
        #     all_actors[i].go_to_location(world.get_random_location_from_navigation())
        #     # random max speed
        #     all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)

        # print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        # while True:
        #     world.wait_for_tick()

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
