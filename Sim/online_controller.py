"""
Script for onlinr simulation of motion patterns.
Simulates and controls multiple vehicles using the LQR controller. 
[Each vehicle with an independent refernce trajectory.]
[All Vehicles spawned at the same time]
[No Rendering possible]
[This script interfaces with the Carla server, receives the states of the actor vehicles and sends the control commands]

Author: Ashish Roongta
SAFE AI lab, Carnegie Mellon University
Copyright @ SafeAI lab-Carnegie Mellon University
"""

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

import sys
sys.path.insert(0,'../GP/')
sys.path.insert(1,'../GP/data_sample')
import pattern_vis


# -------importing the LQR Controller-------------
from online_LQR_Controller import *

class controller2():
    def __init__(self):
        self.no_rendering=False
        self.world=None
        self._synchronous_mode=True
        self.map=None
        self.running=0
        self.num_vehicles=0
        self.m_pat=0
        self.ROI=[[-438,-40],[-438,16],[-301,16],[-301,-40]]    # Defining the Region of Interest

    def _render(self,world):
        '''
        Function to disable rendering
        '''
        settings=world.get_settings()
        settings.no_rendering_mode=True
        world.apply_settings(settings)
    
    def sign(self,a,b,c):
        return (a[0]-c[0])*(b[1]-c[1])-(b[0]-c[0])*(a[1]-c[1])
    

    def inRegion(self,x,a,b,c,d):
        """
        Functiont to check if the point x lies in the polygon with nodes [a,b,c,d]
        cyclic order.
        """
        d1=self.sign(x,a,b)
        d2=self.sign(x,b,c)
        d3=self.sign(x,c,d)
        d4=self.sign(x,d,a)
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
    
        hanger1=[[-261,-40],[-280,-40],[-280,-24],[-261,-24]]
        self.spawn_points=[]
        waypoints=self.map.generate_waypoints(7.0)
        for i,w in enumerate(waypoints):
            x=[w.transform.location.x,-w.transform.location.y]
            # if i==0:
            #     wpts=np.array(x).reshape(1,-1)
            # else:
            #     wpts=np.vstack((wpts,np.array(x).reshape(1,-1)))

            if(self.inRegion(x,hanger1[0],hanger1[1],hanger1[2],hanger1[3])):
                spawn_point=w.transform
                spawn_point.location.z=2
                self.spawn_points.append(spawn_point)
                
        # print('Number of spawn points in the hanger: {}',len(self.spawn_points))
        # np.save('waypoints.npy',wpts)


    def spawn_hangar2(self):
        """
        Function to read and store the spawn points and respective reference trajectories
        from .json files
        """
        file_name="Data/Carla_Town04_T2_Pattern110_FrameEdge_sim_traj.json"
        # file_name='Data/Carla_Town04_T3_Pattern132_FrameEdge_sim_traj.json'
        with open(file_name,'r') as ff:
            a=json.load(ff)
        self.num_vehicles=len(a.keys())

        self.spawn_points=[]
        self.ref_Traj=[]
        self.ref_V=[]
        for car in range(self.num_vehicles):
            # ----loading the reference trajectory-----------------------
            rx=np.array(a[str(car+1)]['x']).reshape(-1,1)
            ry=np.array(a[str(car+1)]['y']).reshape(-1,1)
            ref=np.hstack((rx,ry))

    #         #------Computing the reference velocity--------------
            rx=np.array(a[str(car+1)]['vx']).reshape(-1,1)
            ry=np.array(a[str(car+1)]['vy']).reshape(-1,1)
            ref_v=np.hstack((rx,ry))
            
            self.spawn_points.append(random.choice(self.map.get_spawn_points()))
            self.spawn_points[-1].location.x=ref[0,0]
            self.spawn_points[-1].location.y=ref[0,1]
            self.spawn_points[-1].location.z=2
            self.spawn_points[-1].pitch=0.00193294
            self.spawn_points[-1].rotation.yaw= self.closest_waypt(ref[0,0],ref[0,1])#180*theta/np.pi
            self.spawn_points[-1].rotation.roll=-0.00494385
            self.ref_Traj.append(ref)
            self.ref_V.append(ref_v)


    #         blueprint_library=self.world.get_blueprint_library()
    #         bp=random.choice(blueprint_library.filter('vehicle.toyota.*'))
    #         color=random.choice(bp.get_attribute('color').recommended_values)
    #         bp.set_attribute('color',color)   
    #         try:
    #             self.players.append(self.world.spawn_actor(bp,self.spawn_point))
    #             print('{} Car spawned'.format(self.ind))
    #             car_id=self.players[-1].id
    #             self.controllers[car_id]=Controller2D(self.players[self.ind],ref,ref_v,carla,self.client)
    #             self.v_id.append(car_id)        
    #             self.ind+=1
    #         except RuntimeError:
    #             print('{} Car experienced collision at spawn time'.format(self.ind))
 

    def GP(self, i, car, x, y):
        """
        Taking the current x and y of the vehicle, return reference trajectory and velocity upto next
        horizon.
        """
        traj=self.ref_Traj[i]
        v_ref=self.ref_V[i]

        return traj,v_ref


    def closest_waypt(self,x,y):
        """
        Function to find and return the nearest waypoint orientation
        """
        dist_Sq=np.zeros(len(self.waypts))
        for i,pt in enumerate(self.waypts):
            dist_Sq[i]=(pt.transform.location.x-x)**2+(pt.transform.location.y-y)**2
        
        return self.waypts[np.argmin(dist_Sq)].transform.rotation.yaw

    # def actor_spawn(self):
    #     '''
    #     Function to spawn the actor
    #     '''
    #     spawn_point1=random.choice(self.map.get_spawn_points())  # Randomly choosing a spawn point by querying from the map
        
    #     for car in range(self.num_vehicles):
    #         # ----loading the reference trajectory-----------------------
    #         rx=np.array(self.refA[str(car+1)]['x']).reshape(-1,1)
    #         ry=np.array(self.refA[str(car+1)]['y']).reshape(-1,1)
    #         ref=np.hstack((rx,ry))

    #         #------Computing the reference velocity--------------
    #         rx=np.array(self.refA[str(car+1)]['vx']).reshape(-1,1)
    #         ry=np.array(self.refA[str(car+1)]['vy']).reshape(-1,1)
    #         ref_v=np.hstack((rx,ry))

        
    #         spawn_point1.location.x=ref[0,0]
    #         spawn_point1.location.y=ref[0,1]
    #         spawn_point1.location.z=2
    #         spawn_point1.rotation.pitch=0.00193294
    #         spawn_point1.rotation.yaw= self.closest_waypt(ref[0,0],ref[0,1])#180*theta/np.pi
    #         spawn_point1.rotation.roll=-0.00494385
    #         self.spawn_point=spawn_point1

    #         blueprint_library=self.world.get_blueprint_library()
    #         bp=random.choice(blueprint_library.filter('vehicle.toyota.*'))
    #         color=random.choice(bp.get_attribute('color').recommended_values)
    #         bp.set_attribute('color',color)   
    #         try:
    #             self.players.append(self.world.spawn_actor(bp,self.spawn_point))
    #             print('{} Car spawned'.format(self.ind))
    #             car_id=self.players[-1].id
    #             self.controllers[car_id]=Controller2D(self.players[self.ind],ref,ref_v,carla,self.client)
    #             self.v_id.append(car_id)        
    #             self.ind+=1
    #         except RuntimeError:
    #             print('{} Car experienced collision at spawn time'.format(self.ind))

    def actor_spawn(self):
        # print('number of available spawn points: {}'.format(len(self.spawn_points)))
        blueprint_library=self.world.get_blueprint_library()
        bp=random.choice(blueprint_library.filter('vehicle.toyota.*'))
        # ref=n[0,0]
        # ref_v=[0,0]
        # ref,ref_v=self.GP()
        for car in range(self.num_vehicles):
            color=random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color',color)
        
            try:
            # print(self.spawn_points[car])
                # print(car,self.spawn_points[car])
                self.players.append(self.world.spawn_actor(bp,self.spawn_points[car]))
                print('{} car spawned'.format(self.ind))
                car_id=self.players[-1].id
                self.controllers[car_id]=Controller2D(self.players[self.ind],carla,self.client,self.ROI)
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
        # self.num_vehicles=0
        if self.running==1.0:
            return
        # -----------------------------------
        # self.num_vehicles=1
        self.actor_spawn()

        # m_patterns=["Carla_Town04_T1_Pattern3_Frame33_sim_traj.json","Carla_Town04_T3_Pattern132_Frame133_sim_traj.json","Carla_Town04_T2_Pattern110_Frame11_sim_traj.json"]
        # # pattern_file=m_patterns[self.m_pat]
        # for pattern_file in m_patterns:
        #     f_name='Data/'+pattern_file
        #     with open(f_name,'r') as ff:
        #         self.refA=json.load(ff)
        #     if(pattern_file=="Carla_Town04_T1_Pattern3_Frame33_sim_traj.json"):
        #         self.num_vehicles=2
        #     else:
        #         self.num_vehicles=len(self.refA.keys())   # number of vehicles  
        #     self.actor_spawn()

        self.running=1.0
    
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
            # self.spawn_hangar()
            self.spawn_hangar2()

            while True:
                # self.world.tick()
                # ts=self.world.wait_for_tick()
                try:

                    if self.running==0:
                        self.run_pattern()                     
                    #----------- LQR Control------------------------------
                    try:
                        batch=[]
                        for i, car in enumerate(self.controllers.keys()):
                            x,y,vx,vy=self.controllers[car].update_values()
                            traj,traj_V=self.GP(i, car, x, y)
                            batch.append(self.controllers[car].update_controls(traj,traj_V))

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
