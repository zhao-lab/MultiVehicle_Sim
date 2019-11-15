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
        self.running=0
        self.num_vehicles=0
        self.m_pat=0
        #  loading the reference trajectories for all vehicles........
        # with open('Carla_Town04_T2_Pattern28_sim_traj_31.json','r') as ff:
        #     self.refA=json.load(ff)
        # self.num_vehicles=len(self.refA.keys())   # number of vehicles


    def _render(self,world):
        '''
        Function to disable rendering
        '''
        settings=world.get_settings()
        settings.no_rendering_mode=True
        world.apply_settings(settings)


    def closest_waypt(self,x,y):
        """
        Function to find and return the nearest waypoint orientation
        """
        # print(self.waypts)
        dist_Sq=np.zeros(len(self.waypts))
        for i,pt in enumerate(self.waypts):
            dist_Sq[i]=(pt.transform.location.x-x)**2+(pt.transform.location.y-y)**2
        
        return self.waypts[np.argmin(dist_Sq)].transform.rotation.yaw

    def actor_spawn(self):
        '''
        Function to spawn the actor
        '''
        # if self.ind>=self.num_vehicles:  #...to prevent continuous cycle of vehicle spawns
        #     return

        spawn_point1=random.choice(self.map.get_spawn_points())  # Randomly choosing a spawn point by querying from the map
        
        for car in range(self.num_vehicles):
            # ----loading the reference trajectory-----------------------
            rx=np.array(self.refA[str(car+1)]['x']).reshape(-1,1)
            ry=np.array(self.refA[str(car+1)]['y']).reshape(-1,1)
            ref=np.hstack((rx,ry))

            #------Computing the reference velocity--------------
            rx=np.array(self.refA[str(car+1)]['vx']).reshape(-1,1)
            ry=np.array(self.refA[str(car+1)]['vy']).reshape(-1,1)
            ref_v=np.hstack((rx,ry))

            # ---------Computing the initial yaw orientation for spawning vehicle------
            # theta=np.arctan2((ref[1,1]-ref[0,1]),(ref[1,0]-ref[0,0]))
            # print(self.ind, 180*theta/np.pi)
            # ----Defining the Vehicle Spawn point----------------------------------------
        
            spawn_point1.location.x=ref[0,0]
            spawn_point1.location.y=ref[0,1]
            spawn_point1.location.z=2
            spawn_point1.rotation.pitch=0.00193294
            spawn_point1.rotation.yaw= self.closest_waypt(ref[0,0],ref[0,1])#180*theta/np.pi
            spawn_point1.rotation.roll=-0.00494385
            self.spawn_point=spawn_point1

            blueprint_library=self.world.get_blueprint_library()
            bp=random.choice(blueprint_library.filter('vehicle.toyota.*'))
            color=random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color',color)   
            try:
                self.players.append(self.world.spawn_actor(bp,self.spawn_point))
                print('{} Car spawned'.format(self.ind))
                car_id=self.players[-1].id
                self.controllers[car_id]=Controller2D(self.players[self.ind],ref,ref_v,carla,self.client)
                self.v_id.append(car_id)        
                self.ind+=1
            except RuntimeError:
                print('{} Car experienced collision at spawn time'.format(self.ind))


    def run_pattern(self):
        """
        Function to run the pattern
        """
        self.controllers={}
        self.spawn_point=None
        self.players=[]
        self.ind=0
        self.v_id=[]
        self.num_vehicles=0
        if self.running==1.0:
            return
        m_patterns=["Carla_NGSIM_ROI_LB_Pattern2_Frame110_sim_traj_fac0.json"] #["Carla_Town04_T1_Pattern3_Frame33_sim_traj.json","Carla_Town04_T3_Pattern132_Frame133_sim_traj.json","Carla_Town04_T2_Pattern110_Frame11_sim_traj.json"]
        # pattern_file=m_patterns[self.m_pat]
        for pattern_file in m_patterns:
            f_name='Data/'+pattern_file
            with open(f_name,'r') as ff:
                self.refA=json.load(ff)
            if(pattern_file=="Carla_Town04_T1_Pattern3_Frame33_sim_traj.json"):
                self.num_vehicles=2
            else:
                self.num_vehicles=len(self.refA.keys())   # number of vehicles  
            self.actor_spawn()

        # ------Spawning vehicles-----------
        # spawning all vehicles, one at a time
        # for i in range(self.num_vehicles):
        #     try:
        #         self.actor_spawn()
        #     except RuntimeError:
        #         print('{} Vehicle experienced collision at spawn'.format(i))

        self.running=1.0
        # if (self.m_pat<(len(m_patterns)-1)):
        #     self.m_pat+=1
    
    def destroy_Car(self, car_id):
        """
        Vehicle to remove the 'car_i' car asset fromt the scene and removing it from all
        the lists.
        """
        a=np.asscalar(np.argwhere(np.array(self.v_id)==car_id))
        self.players[a].destroy()
        # car_id=self.v_id[a]
        self.players.remove(self.players[a])
        self.v_id.remove(car_id)
        del self.controllers[car_id]        

        
    def game_loop(self,args):
        '''
        Function to execute the game loop.
        Instantiates the carla world, connects to the client
        '''
        try:
            client=carla.Client(args.host,args.port)
            client.set_timeout(2.0)
            self.client=client
            self.world=client.get_world()  # recieveing the world info from the simulator
            self.map=self.world.get_map()
            
            if self.no_rendering:  # Disabling the rendering 
                self._render(self.world)
            if self._synchronous_mode:   #enabling synchronous mode if selected
                print('enabling synchronous mode')
                settings=self.world.get_settings()
                settings._synchronous_mode=True
                self.world.apply_settings(settings)
            # Generating waypoints
            self.waypts=client.get_world().get_map().generate_waypoints(1.0)
            while True:
                # self.world.tick()
                # ts=self.world.wait_for_tick()
                try:
                    # spawning all vehicles, one at a time
                    # for i in range(self.num_vehicles):
                    #     try:
                    #         self.actor_spawn()
                    #     except RuntimeError:
                    #         print('{} Vehicle experienced collision at spawn'.format(i))

                    if self.running==0:
                        self.run_pattern()                     
                    #----------- LQR Control------------------------------
                    try:
                        batch=[]
                        for car in self.controllers.keys():
                            batch.append(self.controllers[car].update_controls())
                        client.apply_batch_sync([carla.command.ApplyVehicleControl(batch[x][0],batch[x][1]) for x in range(len(batch))])
                        
                        #   Chekcing if any vehicle reached its desination
                        for c in self.controllers.keys():
                            if self.controllers[c].destination==1:
                                print("destroying {} ".format(c)) 
                                self.destroy_Car(c)

                        if(len(self.players)==0):
                            self.running=0
                        # if (np.sum([self.controllers[car].destination for car in self.controllers.keys()])>0):
                        #         # client.apply_batch_sync([carla.command.SetAutopilot(v,True) for v in self.v_id])
                        #     # print("Trajectory Complete detected")
                        #     client.apply_batch_sync([carla.command.DestroyActor(i) for i in self.v_id])
                        #     self.running=0
                    except RuntimeError:
                        pass
                    
                except KeyError:
                    pass
                # self.world.tick()
                # ts=self.world.wait_for_tick()
        finally:
            print("Went into finally")
            for car in self.players:
                car.destroy()
            
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
