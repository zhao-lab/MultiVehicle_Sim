'''
Script to Control Multiple Vehicles using the LQR Controller. 
[Each vehicle with an independent refernce trajectory.]
[Simulation of a single motion pattern]
[No Rendering possible]
[This script interfaces with the Carla server, receives the states of the actor vehicles and sends the control commands]

Author: Ashish Roongta
SafeAI lab
Carnegie Mellon University 
Copyright @ SafeAI lab-Carnegie Mellon University

'''

from __future__ import print_function

import numpy as np
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
import time
import argparse
import random
import json

# -------importing the LQR Controller-------------
from LQR_Controller import *

class controller2():
    def __init__(self):
        self.no_rendering=False
        self.world=None
        self._synchronous_mode=False
        self.map=None
        self.controllers={}
        self.spawn_point=None
        self.players=[]
        self.ind=0
        self.num_vehicles=2  # number of vehicles to spawn and control
        self.system_identification=False
        self.st_time=time.time()
        self.last_sp=0.0
        self._t_diff=0.0   # time difference between spawning consecutive vehicles

        #  loading the reference trajectories for all vehicles........
        with open('GP_sim/Pattern1.json','r') as ff:
            self.refA=json.load(ff)


    def _render(self,world):
        '''
        Function to disable rendering
        '''
        settings=world.get_settings()
        settings.no_rendering_mode=True
        world.apply_settings(settings)

    def actor_spawn(self):
        '''
        Function to spawn the actor
        '''
        spawn_point1=random.choice(self.map.get_spawn_points())  # Randomly choosing a spawn point by querying from the map
        
        # ----loading the reference trajectory-----------------------
        rx=np.array(self.refA[str(self.ind+10)]['x']).reshape(-1,1)
        ry=np.array(self.refA[str(self.ind+10)]['y']).reshape(-1,1)
        ref=np.hstack((rx,ry))

        #------Computing the reference velocity--------------
        rx=np.array(self.refA[str(self.ind+10)]['vx']).reshape(-1,1)
        ry=np.array(self.refA[str(self.ind+10)]['vy']).reshape(-1,1)
        ref_v=np.sqrt(rx**2+ry**2)

        # ---------Computing the initial yaw orientation for spawning vehicle------
        theta=np.arctan((ref[5,0]-ref[0,0])/(ref[5,1]-ref[0,1]))
        print(spawn_point1)

        if self.system_identification:
            spawn_point1.location.x=0
            spawn_point1.location.y=0
            spawn_point1.location.z=2
            spawn_point1.rotation.pitch=0.00193294
            spawn_point1.rotation.yaw=theta
            spawn_point1.rotation.roll=-0.00494385
        else:    
            spawn_point1.location.x=ref[0,0]
            spawn_point1.location.y=ref[0,1]
            spawn_point1.location.z=2
            spawn_point1.rotation.pitch=0.00193294
            spawn_point1.rotation.yaw=1800*theta/np.pi
            spawn_point1.rotation.roll=-0.00494385
        self.spawn_point=spawn_point1

        blueprint_library=self.world.get_blueprint_library()
        bp=random.choice(blueprint_library.filter('vehicle.bmw.grandtourer'))
        color=random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color',color)
        
        self.players.append(self.world.spawn_actor(bp,self.spawn_point))
        print('{} Car spawned'.format(self.ind))
        self.controllers[self.ind]=Controller2D(self.players[self.ind],ref,ref_v,carla)
                
        self.ind+=1


        
    def game_loop(self,args):
        '''
        Function to execute the game loop.
        Instantiates the carla world, connects to the client
        '''
        sys_id=self.system_identification
        try:
            client=carla.Client(args.host,args.port)
            client.set_timeout(2.0)

            self.world=client.get_world()  # recieveing the world info from the simulator
            self.map=self.world.get_map()
            if self.no_rendering:  # Disabling the rendering 
                self._render(self.world)
            if self._synchronous_mode:   #enabling synchronous mode if selected
                print('enabling synchronous mode')
                settings=self.world.get_settings()
                settings._synchronous_mode=True
                self.world.apply_settings(settings)
           
            while True:
                try:
                    t=time.time()-self.st_time   #computing the time passed
                    if len(self.controllers)<self.num_vehicles:  #checking for number of vehicles spawned
                        if t-self.last_sp>self._t_diff:
                            self.actor_spawn()
                            self.last_sp=t
                    
                    #----------- LQR Control------------------------------
                    try:
                        for car in self.controllers.keys():
                            self.controllers[car].update_values()
                            if self.controllers[car].update_controls():
                                self.players[car].destroy()
                                
                                self.controllers.pop(car)
                                print('{} car completed its trajectory'.format(car))
                    except RuntimeError:
                        pass
                except KeyError:
                    pass
        finally:
            print("Went into finally")
            for car in self.controllers.keys():
                self.players[car].destroy()
            
            if self._synchronous_mode:
                settings=self.world.get_settings()
                settings.synchronous_mode=False
                self.world.apply_settings(settings)
            # if (self.world and self.world.recording_enabled):
            #     self.client.stop_recorder()
            # if self.player is not None:
            #     self.player.destroy()
            

def main():
    argparser=argparse.ArgumentParser(description='CARLA Control Client')
    argparser.add_argument('-v', '--verbose',action='store_true',dest='debug',help='print debug information')
    argparser.add_argument('--host',metavar='H',default='127.0.0.1',help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port',metavar='P',default=2000,type=int,help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-a', '--autopilot',action='store_true',help='enable autopilot')
    argparser.add_argument('--res',metavar='WIDTHxHEIGHT',default='1280x720',help='window resolution (default: 1280x720)')
    argparser.add_argument('--filter',metavar='PATTERN',default='vehicle.*',help='actor filter (default: "vehicle.*")')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    # log_level = logging.DEBUG if args.debug else logging.INFO
    # logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    # logging.info('listening to server %s:%s', args.host, args.port)

    # print(__doc__)

    try:
        cnlr=controller2()
        cnlr.game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__=='__main__':
    main()
