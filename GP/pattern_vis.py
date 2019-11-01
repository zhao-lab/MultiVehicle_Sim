#!usr/bin/env python3

import matplotlib.pyplot as plt
import pickle
import numpy as np
from util import Util
from frame import Frame
import copy

# velocity field visualization
u = Util()

with open(u.mix_model_path, "rb") as mix_np:  # load saved mixture model
    mix_model = pickle.load(mix_np)

# with open("data_sample/frames_Uber_od2sq_oneway", "rb") as frame_np: # load saved frames
#     frames = pickle.load(frame_np)
frames = mix_model.frames

pattern_num = np.random.randint(mix_model.K)
pattern_num = 132
frame_pattern_ink = mix_model.frame_ink(pattern_num, 0, True)

pattern_idx = np.asarray(np.where(np.array(mix_model.z) == pattern_num)).ravel()
print('Number of data frames in pattern {}: {}'.format(pattern_num, len(pattern_idx)))
frameTest_idx = np.random.choice(pattern_idx)
# frameTest_idx = 23
print('Simulating pattern {} with Frame {}'.format(pattern_num, frameTest_idx))
frameTest = frames[frameTest_idx]
# frameTest = Frame(np.array([-311.15]),np.array([5.54]),np.array([0]),np.array([0])) # Defining custom test frame
print(frameTest.x)

print(len(frameTest.x))

bk =  mix_model.b[pattern_num]

bk.plotPattern(frameTrain=frame_pattern_ink,frameTest=0,bplotTrainingFrame=False,pattern_num = pattern_num)

# frames2file saves frames to a txt type file with given name
def frames2file(frames,filename):
    traj_data = np.empty((0, 5))
    for j in range(len(frames)):
        traj_data = np.vstack((traj_data,np.array([j*np.ones_like(frames[j].x), frames[j].x , frames[j].y, frames[j].vx, frames[j].vy]).T))
    np.savetxt(filename, traj_data, delimiter=',')


def simulate_traj(mix_model, pattern_num, frame_init, steps=100, bAnimate=1, bDataOut=0):
    bk =  mix_model.b[pattern_num]
    frame_pattern_ink = mix_model.frame_ink(pattern_num, 0, True)
    frames_sim = []
    frame_updated = frame_init
    print('Simulating pattern {}'.format(pattern_num))

    for step in range(steps): # Calculate simulated trajectory
        frame_old = copy.deepcopy(frame_updated)
        frame_updated = bk.updateFramebyPattern(frame_pattern_ink,frame_updated)
        frames_sim.append(Frame(frame_old.x, frame_old.y,frame_updated.vx, frame_updated.vy))
        del frame_old
    frames_sim.append(Frame(frame_updated.x, frame_updated.y,np.zeros_like(frame_updated.vx), np.zeros_like(frame_updated.vy)))

    if bAnimate == 1: # Display animation of simulated trajectory
        Frame.animate_frames(frames_sim, 0.01)
    else: # Display plot of simulated trajectory
        plt.scatter(frames_sim[0].x, frames_sim[0].y,color='k')
        for veh_idx in range(len(frames_sim[0].x)):
            x_traj = []
            y_traj = []
            for j in range(len(frames_sim)):
                x_traj.append(frames_sim[j].x[veh_idx])
                y_traj.append(frames_sim[j].y[veh_idx])
            plt.plot(x_traj,y_traj,color = 'g')
        plt.xlim([u.roi_xMin, u.roi_xMax])
        plt.ylim([u.roi_yMin, u.roi_yMax])
        plt.title('Simulated trajectory for pattern {}'.format(pattern_num))
        plt.show()

    if bDataOut == True: # Save simulated trajectory
        filename = 'GP-sim/{}3_Pattern{}_FrameEdge_sim_traj.xlsx'.format(u.dataset,pattern_num,frameTest_idx)
        frames2file(frames_sim, filename)


simulate_traj(mix_model, pattern_num, frameTest, steps=300,bAnimate=0,bDataOut=1)

print('Visualization Finished') 