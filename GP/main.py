#!usr/bin/env python3

import matplotlib.pyplot as plt
import pickle
from mixtureModel import MixtureModel
import time
from util import Util


# Main script for DP-GP
# The python version code is implemented by Vinay Varma Kalidindi, Mengdi Xu @SafeAI lab in CMU.
# Initial python code by Yaohui Guo and Vinay Varma Kalidindi
# paper reference:
# Modeling Multi-Vehicle Interaction Scenarios Using Gaussian Random Field
# https://arxiv.org/pdf/1906.10307.pdf
#
# Improvement:
# The code structure is more clear and can easily be implemented for various applications.
#
# Thanks members of SafeAI lab for discussion.
#
# Input:
# frames: list with element as object defined in frame.py
# Output:
# Mixture model as defined in mixtureModel.py


start_time = time.time()
bNewProgram = True
u = Util()
n_iter = 2  # Gibbs Sampling Iterations

if __name__=='__main__':
    
    if bNewProgram == True:
        # Initialize the mixture model
        print('------------Initializing new mixture model------------')
        with open(u.frames_path, "rb") as fb: # load saved frames
            load_frames = pickle.load(fb)

        a = MixtureModel(load_frames)
        a.mixture_model()
        
        if u.save_mix_model_init:
            # save the initialized mixture model
            with open(u.mix_model_path, "wb") as mm:
                pickle.dump(a, mm)
    else:
        # load the saved mixture model
        print('------------Loading existing mixture model------------')
        with open(u.mix_model_path, "rb") as mm:  # 
            a = pickle.load(mm)

    print('------------Gibbs Sampling------------')
    for iter in range(n_iter):
        # update each frame's indicator/assignmentload saved mixture model
        a.update_all_assignment()
        # update gaussian process patterns
        a.update_all_pattern()
        # show number of mixtures
        a.show_mixture_model()
        # save mixture model
        with open(u.mix_model_path, "wb") as mm:
            pickle.dump(a, mm)

    print('DPGP finished!!')
    print("--------------- %s ----------------" % (time.time() - start_time))