"""
Script to pre-process the data file. Removes the data of actors that are outside 
the ROI. 
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from math import sqrt
def pre_process(file_name):
    data_dir = '../../Data_Record/'
    data = pd.read_csv(data_dir + file_name).values
    ROI_Center = [-2.35, -189.8]
    radius = 30
    flg = 0
    for i in range(data.shape[0]):
        # print(sqrt((ROI_Center[0]- data[i,4])**2 +(ROI_Center[1]-data[i,5])**2))
        if not(sqrt((ROI_Center[0] + data[i,4])**2 +(ROI_Center[1]+data[i,5])**2) <= radius):
            continue
        if(flg == 0):
            p_data = data[i,:].reshape(1,-1)
            flg = 1
        else:
            p_data = np.vstack((p_data, data[i,:].reshape(1,-1)))

    # .... Saving the processed data file...........
    heading="Frame,sTime,VehicleID,EGO,X,Y,Z,Roll,Pitch,Yaw,Vx,Vy,Vz,Ax,Ay,Az,Wx,Wy,Wz,Throttle,Steer,Brake"
    data_file = os.path.join('Data/',"processed_"+file_name)
    np.savetxt(data_file, p_data, delimiter=',', header=heading)
    print('{} Saved'.format(data_file))

if __name__=='__main__':
    ff = 'Town4_Tintersect.csv'
    pre_process(ff)
