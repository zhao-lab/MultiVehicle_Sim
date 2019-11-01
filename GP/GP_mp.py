#!usr/bin/env python3
"""
Script to interact with Carla Simulator. 
At each timestep of simulation, takes in curent vehicle states (x,y,vx,vy) and
returns the reference trajectory & reference velocity.
The reference velocity for each vehicle is computed via pattern assigment followed by 
GP posterior. The reference trajectory is computed byt rajectory roll-out.

"""
import matplotlib.pyplot as plt
import pickle
import numpy as np
from util import Util
from frame import Frame
import copy

def GP_sim(vehicle_states, patternN=132, steps=100):
    """
    Function to communicate between GP and Carla.
    """   
    # Instantiating the velocity field visualization
    u = Util()

    with open(u.mix_model_path, "rb") as mix_np:  # load saved mixture model
    mix_model = pickle.load(mix_np)

    # with open("data_sample/frames_Uber_od2sq_oneway", "rb") as frame_np: # load saved frames
    #     frames = pickle.load(frame_np)
    frames = mix_model.frames

    pattern_num = np.random.randint(mix_model.K)
    pattern_num = patternN
    frame_pattern_ink = mix_model.frame_ink(pattern_num, 0, True)

    pattern_idx = np.asarray(np.where(np.array(mix_model.z) == pattern_num)).ravel()
    # print('Number of data frames in pattern {}: {}'.format(pattern_num, len(pattern_idx)))
    # frameTest_idx = np.random.choice(pattern_idx)
    # frameTest_idx = 23
    print('Simulating pattern {} with Frame {}'.format(pattern_num, frameTest_idx))
    # frameTest = frames[frameTest_idx]
    # frameTest = Frame(np.array([-311.15]),np.array([5.54]),np.array([0]),np.array([0])) # Defining custom test frame
    # print(frameTest.x)
    frameTest=vehicle_states


    bk =  mix_model.b[pattern_num]

    # bk.plotPattern(frameTrain=frame_pattern_ink,frameTest=0,bplotTrainingFrame=False,pattern_num = pattern_num)

    # frames2file saves frames to a txt type file with given name
    def frames2file(frames,filename):
        traj_data = np.empty((0, 5))
        for j in range(len(frames)):
            traj_data = np.vstack((traj_data,np.array([j*np.ones_like(frames[j].x), frames[j].x , frames[j].y, frames[j].vx, frames[j].vy]).T))
        np.savetxt(filename, traj_data, delimiter=',')


    # steps=100, 
    bAnimate=1 
    bDataOut=0

    #-------Simulating and Rolling out Trajectories---------------------------

    bk =  mix_model.b[pattern_num]
    frame_pattern_ink = mix_model.frame_ink(pattern_num, 0, True)
    frames_sim = []
    frame_updated = frameTest
    
    vehicles={}
    vehicles['x']=frameTest.x.reshape(-1,1)
    vehicles['y']=frameTest.y.reshape(-1,1)
    vehicles['vx']=frameTest.vx.reshape(-1,1)
    vehicles['vy']=frameTest.vy/reshape(-1,1)

    # print('Simulating pattern {}'.format(pattern_num))

    for step in range(steps): # Calculate simulated trajectory
        frame_old = copy.deepcopy(frame_updated)
        frame_updated = bk.updateFramebyPattern(frame_pattern_ink,frame_updated)
        # Storing the computed (x,y,vx,vy) for each vehicle in the 'vehicles' hash-map
        vehicles['x']=np.hstack((vehicles['x'],frame_old.x.reshape(-1,1)))
        vehicles['y']=np.hstack((vehicles['y'],frame_old.y.reshape(-1,1)))
        vehicles['vx']=np.hstack((vehicles['vx'],frame_updated.vx.reshape(-1,1)))
        vehicles['vy']=np.hstack((vehicles['vy'],frame_updated.vy.reshape(-1,1)))
        frames_sim.append(Frame(frame_old.x, frame_old.y,frame_updated.vx, frame_updated.vy))
        del frame_old
    frames_sim.append(Frame(frame_updated.x, frame_updated.y,np.zeros_like(frame_updated.vx), np.zeros_like(frame_updated.vy)))
    vehicles['x']=np.hstack((vehicles['x'],frame_updated.x.reshape(-1,1)))
    vehicles['y']=np.hstack((vehicles['y'],frame_updated.y.reshape(-1,1)))
    vehicles['vx']=np.hstack((vehicles['vx'],np.zeros_like(frame_updated.vx).reshape(-1,1)))
    vehicles['vy']=np.hstack((vehicles['vy'],np.zeros_like(frame_updated.vy).reshape(-1,1)))
    
    return vehicles
    # if bAnimate == 1: # Display animation of simulated trajectory
    #     Frame.animate_frames(frames_sim, 0.01)
    # else: # Display plot of simulated trajectory
    #     plt.scatter(frames_sim[0].x, frames_sim[0].y,color='k')
    #     for veh_idx in range(len(frames_sim[0].x)):
    #         x_traj = []
    #         y_traj = []
    #         for j in range(len(frames_sim)):
    #             x_traj.append(frames_sim[j].x[veh_idx])
    #             y_traj.append(frames_sim[j].y[veh_idx])
    #         plt.plot(x_traj,y_traj,color = 'g')
    #     plt.xlim([u.roi_xMin, u.roi_xMax])
    #     plt.ylim([u.roi_yMin, u.roi_yMax])
    #     plt.title('Simulated trajectory for pattern {}'.format(pattern_num))
    #     plt.show()

    # if bDataOut == True: # Save simulated trajectory
    #     filename = 'GP-sim/{}3_Pattern{}_FrameEdge_sim_traj.xlsx'.format(u.dataset,pattern_num,frameTest_idx)
    #     frames2file(frames_sim, filename)


simulate_traj(mix_model, pattern_num, frameTest, steps=300,bAnimate=0,bDataOut=1)

print('Visualization Finished') 