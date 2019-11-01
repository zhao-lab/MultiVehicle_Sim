#!usr/bin/env python3

import numpy as np
from scipy.stats import gamma


# Util is a class with predefined parameters in DP-GP

class Util(object):
    def __init__(self):
        self.gammaShape = 10.0 # gamma shape parameter for draw w
        self.gammaScale = 2.0 # gamma scale parameter for draw w
        self.eip_prior = 0.000000001 # noise scale parameter for prior
        self.eip_post = 0.0001 # noise scale parameter for posterior
        self.N_nbr_num = 5 # number of nearest neighbors in knn
        self.mc_iteration = 100 # MC iteration for unseen pattern likelihood calculation
        self.useMLE = False # whether use Maximum likelihood for update wx and wy
        self.assignment_MAP = True # whether use Maximum a posterior to update z(assignment)
        self.alpha_MAP = True # whether use Maximum a posterior to update alpha
        self.show_MM = True # show info of mixture models
        self.max_obj_update = 200 # max number of data points used to update each motion pattern
        self.alpha_max = 20 # max boundary of alpha
        self.alpha_min = 0.001 # min boundary of alpha
        self.alpha_sample_size = 200 # sample number between max and min boundary of alpha
        self.alpha_initial_sample = True # whether to use random sample from inverse gamma distribution
        self.alpha_initial = 1 # predefine initial alpha if not sample
        self.sigman = 0.2 # noise covariance scalar in kernel function
        self.save_mix_model_init = True # save initialized mixture model
        

        self.dataset = 'Carla_Town04_T'
        
        if self.dataset == 'NGSIM_highway': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = 60
            self.roi_xMin = 0
            self.roi_yMax = 1350
            self.roi_yMin = 1550

            self.delT_data = 0.1

        if self.dataset == 'NGSIM_intersection': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = 100
            self.roi_xMin = -100
            self.roi_yMax = 600
            self.roi_yMin = 50

            self.delT_data = 0.1

            self.frames_path = "data_sample/frames_NGSIM_intersection"
            self.mix_model_path = "data_sample/NGSIM_intersection_final_DPGP2"


        if self.dataset == 'Sumo_left': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = 65
            self.roi_xMin = 0
            self.roi_yMax = 100
            self.roi_yMin = 15

            self.delT_data = 0.1

            self.frames_path = "data_sample/frames_left"
            self.mix_model_path = "data_sample/Sumo_left_final_DPGP"

        if self.dataset == 'Carla_Town03': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = 255
            self.roi_xMin = 190
            self.roi_yMax = 120
            self.roi_yMin = 15

            self.delT_data = 0.1

            self.frames_path = "data_sample/frames_Carla03"
            self.mix_model_path = "data_sample/Carla03_final_DPGP"


        if self.dataset == 'Carla_Town04': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = -301
            self.roi_xMin = -428
            self.roi_yMax = 40
            self.roi_yMin = -16

            self.delT_data = 0.1

            self.frames_path = "data_sample/frames_Carla04"
            self.mix_model_path = "data_sample/Carla04_final_DPGP"


        if self.dataset == 'Carla_NGSIM_LB': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = -1650
            self.roi_xMin = -1760
            self.roi_yMax = 1950
            self.roi_yMin = 1860

            self.delT_data = 0.1

            self.frames_path = "data_sample/frames_Carla_NGSIM_LB"
            self.mix_model_path = "data_sample/Carla_NGSIM_LB_final_DPGP"


        if self.dataset == 'Ped_biwi': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = 4
            self.roi_xMin = -2
            self.roi_yMax = 4
            self.roi_yMin = -10

            self.delT_data = 0.4

            self.frames_path = "data_sample/frames_ped_biwi"
            self.mix_model_path = "data_sample/Ped_biwi_final_DPGP"


        if self.dataset == 'Ped_zara': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = 15
            self.roi_xMin = 0
            self.roi_yMax = 12
            self.roi_yMin = 0

            self.delT_data = 0.4

            self.frames_path = "data_sample/frames_ped_zara"
            self.mix_model_path = "data_sample/Ped_zara_final_DPGP"


        if self.dataset == 'Carla_Town04_T': 
            #Region of interest boundaries (Rectangular)    
            self.roi_xMax = -301
            self.roi_xMin = -428
            self.roi_yMax = 40
            self.roi_yMin = -16

            self.delT_data = 0.1

            self.frames_path = "/home/gauss/Carla_096/TRI_MultiVehicle_Sim/GP/data_sample/frames_Carla04_T3"
            self.mix_model_path = "/home/gauss/Carla_096/TRI_MultiVehicle_Sim/GP/data_sample/Carla04_T3_final_DPGP"

    def draw_w(self):
        # randomly draw wx and wy from gamma distribution
        wx = np.random.gamma(self.gammaShape, self.gammaScale)
        wy = np.random.gamma(self.gammaShape, self.gammaScale)
        pwx = gamma.pdf(wx, a=self.gammaShape, scale=self.gammaScale) # prob of wx
        pwy = gamma.pdf(wy, a=self.gammaShape, scale=self.gammaScale) # prob of wy
        return wx, wy, pwx, pwy

