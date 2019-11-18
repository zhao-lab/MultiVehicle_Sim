"""
Script to Query state information from all the actors, from the Carla Server in real time

Author: Ashish Roongta
SafeAI lab, Carengie Mellon University
Copyright @ SafeAI lab, Carnegie Mellon University
"""

import os
import numpy as np
class dataQ():

	def __init__(self,hud):
		############File_argumensts####################################
		self.date='NGSim_ROI_const'
		self.test='LB'                      # indication of the experiment number
		self.data_dir='../../Data_Record/'    #File directory


		###############################################################
		self.HUD=hud
		self.actors={}
		self.ego_vehicles=[]
		self.start_time=self.HUD.simulation_time
		self.attr_num=22


	def data_manage(self,actor):
		if not(self.actors):
			self.start_time=self.HUD.simulation_time
		_id=actor.id
		tranform=actor.get_transform()
		location=tranform.location
		rotation=tranform.rotation
		frame=self.frame
		velocity=actor.get_velocity()
		accl=actor.get_acceleration()
		Wvel=actor.get_angular_velocity()
		control=actor.get_control()
		time=round(self.HUD.simulation_time-self.start_time,3)
		if _id in self.ego_vehicles:
			hero=1
		else:
			hero=0 
		# ----Checking for ego-vehciles----------------
		if actor.attributes['role_name']=='hero':
			if not(_id in self.ego_vehicles):
				self.ego_vehicles.append(_id)

		if not(_id in self.actors):
			self.actors[_id]=np.empty((0,self.attr_num))
		self.actors[_id]=np.vstack((self.actors[_id],[frame,time,_id,hero,location.x,location.y,location.y,rotation.pitch,rotation.yaw,rotation.roll,
			velocity.x,velocity.y,velocity.z,accl.x,accl.y,accl.z,Wvel.x,Wvel.y,Wvel.z,control.throttle,control.steer,
			control.brake]))
		pass
	def data_input(self,actor_list,frame):
		'''
		Function to take the actor_list from Carla server and work on it
		[input]
		*actor_list: list of all the actors in the scene at each instance
		[output]

		>>actor_list=client.get_world().get_actors()   To generate the actor_list in game_loop

		'''
		self.frame=frame
		vehicle_list=[actor for actor in actor_list.filter('vehicle.*')]
		for vehicle in vehicle_list:
			self.data_manage(vehicle)
		# print('number of vehicles: {}'.format(len(vehicle_list)))
			# print(actors)
		# for actor in vehcile_list:

		# 	print("id: {}".format(actor.id)," Type: {}".format(actor.type_id)," location: {}".format(actor.get_location())," speed: {}".format(actor.get_velocity()))
		# pass
	def data_save(self):
		'''
		Function to save the data files created
		'''
		heading="Frame,sTime,VehicleID,EGO,X,Y,Z,Roll,Pitch,Yaw,Vx,Vy,Vz,Ax,Ay,Az,Wx,Wy,Wz,Throttle,Steer,Brake"
		self.data=np.empty((0,self.attr_num))
		for _id in self.actors:
			self.data=np.vstack((self.data,self.actors[_id]))

		file_name=os.path.join(self.data_dir,self.date+"_"+self.test+".csv")
		os.makedirs(os.path.dirname(self.data_dir),exist_ok=True)    #Make the directory if it doesn't exist
		np.savetxt(file_name,self.data,delimiter=',',header=heading)
		print(file_name, "Saved")

