'''
Script to Control Multiple Vehicles using the LQR Controller. 
[Each vehicle with an independent refernce trajectory.]
[All Vehicles spawned at the same time]
[No Rendering possible]
[This script interfaces with the Carla server, receives the states of the actor vehicles and sends the control commands]

Author: Ashish Roongta
SafeAI lab
Carnegie Mellon University
Copyright @ SafeAI lab-Carnegie Mellon University
'''

from __future__ import print_function

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


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import time
import argparse
import random
import json


# -------importing the LQR Controller-------------
from LQR_Controller import *
# from PP_Controller import *
# ------importing plotter-----------------
# from plot_states import *

class controller2():
    def __init__(self):
        self.no_rendering=False
        self.world=None
        self._synchronous_mode=True
        self.map=None
        self.controllers={}
        self.spawn_point=None
        self.players=[]
        self.ind=0
        self.v_id=[]
        self.it=0
        #  loading the reference trajectories for all vehicles........
        with open('GP_leftTurn1_traj.json','r') as ff:
            self.refA=json.load(ff)
        self.num_vehicles=len(self.refA.keys())   # number of vehicles


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
        if self.ind>=self.num_vehicles:  #...to prevent continuous cycle of vehicle spawns
            return

        spawn_point1=random.choice(self.map.get_spawn_points())  # Randomly choosing a spawn point by querying from the map
        
        # ----loading the reference trajectory-----------------------
        rx=-np.array(self.refA[str(self.ind+1)]['x']).reshape(-1,1)
        ry=np.array(self.refA[str(self.ind+1)]['y']).reshape(-1,1)
        ref=np.hstack((rx,ry))

        #------Computing the reference velocity--------------
        rx=np.array(self.refA[str(self.ind+1)]['vx']).reshape(-1,1)
        ry=np.array(self.refA[str(self.ind+1)]['vy']).reshape(-1,1)
        ref_v=np.sqrt(rx**2+ry**2)

        # ---------Computing the initial yaw orientation for spawning vehicle------
        theta=np.arctan2((ref[1,1]-ref[0,1]),(ref[1,0]-ref[0,0]))
        # print(self.ind, 180*theta/np.pi)
        # ----Defining the Vehicle Spawn point----------------------------------------
    
        spawn_point1.location.x=ref[0,0]
        spawn_point1.location.y=ref[0,1]
        spawn_point1.location.z=2
        spawn_point1.rotation.pitch=0.00193294
        spawn_point1.rotation.yaw=180*theta/np.pi
        spawn_point1.rotation.roll=-0.00494385
        self.spawn_point=spawn_point1

        blueprint_library=self.world.get_blueprint_library()
        bp=random.choice(blueprint_library.filter('vehicle.bmw.grandtourer'))
        color=random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color',color)   
        self.players.append(self.world.spawn_actor(bp,self.spawn_point))
        print('{} Car spawned'.format(self.ind))
        self.controllers[self.ind]=Controller2D(self.players[self.ind],ref,ref_v,carla)
        self.v_id.append(self.players[-1].id)        
        self.ind+=1

    def batch_spawn(self):
        """
        Function to spawn actors in batch and generate velocity array for batch mode
        """
        spawn_point1=random.choice(self.map.get_spawn_points())  # Randomly choosing a spawn point by querying from the map
        self.spawn_points=[]
        self.ref_v={}
        for i in range(self.num_vehicles):
            # ----loading the reference trajectory-----------------------
            rx=-np.array(self.refA[str(self.ind+1)]['x']).reshape(-1,1)
            ry=np.array(self.refA[str(self.ind+1)]['y']).reshape(-1,1)
            ref=np.hstack((rx,ry))

            #------Computing the reference velocity--------------
            rx=self.refA[str(self.ind+1)]['vx']
            ry=self.refA[str(self.ind+1)]['vy']
            # ref_v=np.sqrt(rx**2+ry**2)

            # ---------Computing the initial yaw orientation for spawning vehicle------
            theta=np.arctan2((ref[1,1]-ref[0,1]),(ref[1,0]-ref[0,0]))
            # print(self.ind, 180*theta/np.pi)
            # ----Defining the Vehicle Spawn point----------------------------------------

            spawn_point1.location.x=ref[0,0]
            spawn_point1.location.y=ref[0,1]
            spawn_point1.location.z=2
            spawn_point1.rotation.pitch=0.00193294
            spawn_point1.rotation.yaw=180*theta/np.pi
            spawn_point1.rotation.roll=-0.00494385

            self.spawn_points.append(spawn_point1)
            self.ref_v[i]=[]
            for j in range(len(rx)):
                vb=carla.Vector3D(-rx[j],-ry[j],0)
                self.ref_v[i].append(vb)
        for i in range(self.num_vehicles):
            self.actor_spawn()
        # print('done')
        # blueprint_library=self.world.get_blueprint_library()
        # bp=random.choice(blueprint_library.filter('vehicle.bmw.grandtourer'))
        # color=random.choice(bp.get_attribute('color').recommended_values)
        # bp.set_attribute('color',color)   
    
        # for i in range(self.num_vehicles):

        #     try:
        #         self.players.append(self.world.spawn_actor(bp,self.spawn_points[i]))
        #         print('{} Car spawned'.format(i))
        #         self.controllers[self.ind]=Controller2D(self.players[self.ind],ref,self.ref_v,carla)
        #         self.v_id.append(self.players[-1].id)        
        #         # self.ind+=1
        #     except RuntimeError:
        #         print('Vehicle {} faced collision at spawn point'.format(i))

        
    def game_loop(self,args):
        '''
        Function to execute the game loop.
        Instantiates the carla world, connects to the client
        '''
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
            
            self.it+=1    
            while True:
                # self.world.tick()
                # ts=self.world.wait_for_tick()
                try:

                    # # spawning all vehicles, one at a time
                    # for i in range(self.num_vehicles):
                    #     try:
                    #         self.actor_spawn()
                    #     except RuntimeError:
                    #         print('{} Vehicle experienced collision at spawn'.format(i))
                    
                    self.batch_spawn()
                    
                    #----------- LQR Control------------------------------
                    try:
                        # batch=[]
                        # for car in self.controllers.keys():
                        #     batch.append(self.controllers[car].update_controls())
                        # client.apply_batch_sync([carla.command.ApplyVehicleControl(batch[x][0],batch[x][1]) for x in range(len(batch))])
                        client.apply_batch([carla.command.ApplyVelocity(self.v_id[v],self.ref_v[v][self.it]) for v in range(self.num_vehicles)])
                        
                        # for car in self.controllers.keys():
                        #     self.controllers[car].update_values()
                        #     if self.controllers[car].update_controls():
                        #         self.players[car].destroy()
                                
                        #         self.controllers.pop(car)
                        #         print('{} car completed its trajectory'.format(car))
                    except RuntimeError:
                        pass
                
                except KeyError:
                    pass
                # self.world.tick()
                # ts=self.world.wait_for_tick()
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
