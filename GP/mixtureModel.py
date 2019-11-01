#!usr/bin/env python3

import numpy as np
import multiprocessing as mp
from functools import partial
from sklearn.neighbors import NearestNeighbors
from scipy.special import gamma
from scipy.stats import invgamma

from frame import Frame
from util import Util
from MotionPattern import MotionPattern


# MixtureModel is a class with functions to
# a) update motion pattern represented by Gaussian Process
# b) update non-parametric bayesian model represented by Dirichlet Process


class MixtureModel(object):
    def __init__(self, frames):
        self.b = []  # Motion Patterns, list
        self.z = []  # Assignments , list
        self.K = None  # Number of Motion Patterns, scalar
        self.n = None  # number of Data Points
        self.alpha = None  # concentration parameter
        self.partition = []  # number of frames in each cluster, list
        self.parameterIteration = None  # Iteration of Gibbs Sampling
        self.assignmentIteration = None  # Iteration of Gibbs Sampling
        self.Util = Util()
        self.frames = frames

    def mixture_model(self):
        self.assignmentIteration = 0
        self.parameterIteration = 0
        if self.Util.alpha_initial_sample:
            # concentration parameter prior: inverse gamma function with scale parameter: 1
            self.alpha = invgamma.rvs(a=1, size=1)
        else:
            self.alpha = self.Util.alpha_initial
        self.n = 1
        self.K = 1
        self.z.append(0)
        # draw a new pattern for the first frame
        wx, wy, pwx, pwy = self.Util.draw_w()
        frame_first = []
        frame_first.append(self.frames[0])
        ux, uy, sigmax, sigmay, sigman = self.cov_mean(frame_first)
        pattern_0 = MotionPattern(ux, uy, sigmax, sigmay, sigman, wx, wy)
        pattern_0.update_para(self.frames[0])
        self.b.append(pattern_0)
        # update partition
        self.partition = self.z2partition(self.z, self.K)

        # from the second frame, do update assignment
        for i in range(1, len(self.frames)):
            self.n = i + 1
            # first assign unseen frame to cluster 0
            self.z.append(0)
            # update assignment of frame i
            self.update_assignment(i)
            print('Initializing Frame: ', i, 'total frames: ', len(self.frames))

    def update_all_pattern(self):
        # update parameter of each motion pattern
        frame_ink_prep = [None] * self.K
        b_prep = [None] * self.K
        for k in range(self.K):
            frame_ink_prep[k] = self.frame_ink(k, 0, True)
            b_prep[k] = self.b[k]
        # update patterns in parallel processing style
        pool = mp.Pool(mp.cpu_count())
        b_prep = [pool.apply(self.update_one_pattern, args=(frame_ink_prep[k], b_prep[k]))
                  for k in range(self.K)]
        pool.close()
        for k in range(self.K):
            self.b[k] = b_prep[k]

        # update concentration parameter
        self.draw_alpha()
        self.parameterIteration += 1

    def update_one_pattern(self, frame_ink_k, b_prep_k):
        # extract frames from one pattern and recalculate the pattern parameters
        frame_ink = frame_ink_k
        if len(frame_ink.x) > self.Util.max_obj_update:
            idx = np.random.randint(len(frame_ink.x), size=self.Util.max_obj_update)
            frame_ink = Frame(frame_ink.x[idx], frame_ink.y[idx], frame_ink.vx[idx], frame_ink.vy[idx])
        b_prep_k = b_prep_k.update_para(frame_ink)
        return b_prep_k

    def draw_new_pattern(self):
        # draw a new motion pattern from the current mixture model
        # p_pattern is the prob of this model
        wx, wy, pwx, pwy = self.Util.draw_w()
        new_pattern = MotionPattern(self.b[0].ux, self.b[0].uy, self.b[0].sigmax,
                                    self.b[0].sigmay, self.b[0].sigman, wx, wy)
        p_pattern = pwx * pwy
        return new_pattern, p_pattern

    def draw_alpha(self):
        # note that this part contains a lot of pre-defined parameters stored in Util.
        logp = []
        # update alpha in parallel processing style
        pool = mp.Pool(mp.cpu_count())
        candidate = np.linspace(self.Util.alpha_min, self.Util.alpha_max, self.Util.alpha_sample_size)
        logp = pool.map(self.p_alpha, (c for c in candidate))
        pool.close()
        # normalization
        logp = logp - np.max(logp)
        if self.Util.alpha_MAP:
            idx = np.argmax(logp)
            self.alpha = candidate[idx]
            print('max idx', idx)
            print('alpha', self.alpha)
        else:
            p = np.exp(logp)
            self.alpha = np.random.choice(candidate, 1, p)

    def p_alpha(self, alpha):
        # log of the parameter posterior probability
        t1 = np.log(alpha)*(self.K - 1.5)
        t2 = - 0.5/alpha
        t3 = -np.sum(np.log(np.arange(alpha,alpha+self.n)))
        p = t1 + t2 + t3
        return p

    def update_all_assignment(self):
        # update assignments for all frames
        for i in range(self.n):
            self.update_assignment(i)
        self.assignmentIteration += 1

    def update_assignment(self, i):
        print('updating assignment: ', i)
        zi_old = self.z[i]
        # posterior probability of frame i belongs to each pattern
        log_pzik_post = self.assignment_posterior(i)

        # reassign zi
        if self.Util.assignment_MAP:
            self.z[i] = np.argmax(log_pzik_post)
        else:
            # not fully check here
            log_pzik_post = log_pzik_post - np.max(log_pzik_post)
            pzik_post = np.exp(log_pzik_post)
            pzik_post = pzik_post/np.sum(pzik_post)
            candidate = np.linspace(0, self.K, self.K+1)
            self.z[i] = np.random.choice(candidate, 1, pzik_post)

        # if zi = K+1, draw a new pattern
        if self.z[i] == self.K:
            print('update k and b')
            new_pattern, p_pattern = self.draw_new_pattern()
            # unlike the existing motion pattern, the parameter of
            # the new patterns are updated once generated
            self.b.append(new_pattern.update_para(self.frames[i]))
            self.K += 1

        # if b(zi_old) is empty after the frame_i left
        all_z = set(self.z)
        if not zi_old in all_z:
            print('***********************************')
            print('delete pattern: ', zi_old)
            print('***********************************')
            # if is empty, delete the pattern
            del self.b[zi_old]
            other_z = np.argwhere(np.array(self.z) > zi_old).astype(int)
            for m in range(len(other_z)):
                self.z[other_z[m][0]] -= 1
            self.K -= 1

        # update partition
        self.partition = self.z2partition(self.z, self.K)

        # update the unchanged pattern parameters
        # TODO structure improvement: parallel processing
        for k in range(self.K):
            idx = np.where(np.array(self.z) == k)
            idx = np.asarray(idx)
            idx = idx[0].astype(int)
            list_f = []
            for idx_iter in range(len(idx)):
                list_f.append(self.frames[idx[idx_iter]])
            ux, uy, sigmax, sigmay, sigman = self.cov_mean(frames=list_f)

            self.b[k].ux = ux
            self.b[k].uy = uy
            self.b[k].sigmax = sigmax
            self.b[k].sigmay = sigmay
            self.b[k].sigman = sigman

        if self.Util.show_MM:
            self.show_mixture_model()

    def assignment_posterior(self, i):
        # calculate the posterior distribuion of zi
        # calculate the PMF vector

        # for existing patterns
        pool = mp.Pool(mp.cpu_count())
        log_post_i = partial(self.log_posterior_exist_pattern, i=i)
        log_pzik_post = pool.map(log_post_i, (k for k in range(self.K)))
        pool.close()
        # for unseen patterns
        log_pzik_post.append(self.log_posterior_unseen_pattern(i))
        
        print(log_pzik_post)
        return log_pzik_post

    def log_posterior_unseen_pattern(self, i):
        # calculate the posterior given frames and parameters of each pattern
        # prior
        log_pzik_new_prior = np.log(self.alpha/(self.n - 1 + self.alpha))
        # likelihood : the MCMC integration
        log_pzik_likelihood = self.log_likelihood_unseen_pattern(i)
        # print('i,k+1 prior :{} likel: {} '.format(log_pzik_new_prior, log_pzik_likelihood))
        return log_pzik_new_prior + log_pzik_likelihood

    def log_likelihood_unseen_pattern(self, i):
        # use MC to calculate the integration
        temp_sum = 0
        frame_i = self.frames[i]
        # TODO: check whether can be paralleled
        for i in range(self.Util.mc_iteration):
            new_pattern, p_pattern = self.draw_new_pattern()
            likelihood = new_pattern.GP_prior(frame_i)
            temp_sum += likelihood*p_pattern
        integration = temp_sum/self.Util.mc_iteration
        return np.log(integration)

    def log_posterior_exist_pattern(self, k, i):
        # calculate the posterior PDF of log_pzik_prior
        partition_without_i = self.partition.copy()
        partition_without_i[self.z[i]] -= 1
        if partition_without_i[k] == 0:
            print('frame{} in pattern{} Returning -inf'.format(i,k))
            return np.array(float("-inf"))
        else:
        # if True:
            # when p near 0, log will be -inf which arises error
            log_pzik_prior = np.log(partition_without_i[k]/(self.n - 1 + self.alpha))
            # print(partition_without_i[k]/(self.n - 1 + self.alpha))
            log_pzik_likelihood = self.log_likelihood_exit_pattern(i, k)
            # print('frame{} in pattern{} prior :{} likel: {}'.format(i, k, log_pzik_prior, log_pzik_likelihood))
            return log_pzik_prior + log_pzik_likelihood
        

    def log_likelihood_exit_pattern(self, i, k):
        # calculate the log likelihood of frame i under a given pattern k
        # check if frame i is the only frame in bk
        if self.z[i] == k and self.partition[k] == 1:
            log_pzik_likelihood = np.log(self.b[k].GP_prior(self.frames[i]))
            # print('frame{} in pattern{} GP prior frame_ink likel: {}'.format(i, k, log_pzik_likelihood))
            return log_pzik_likelihood
        else:
            frame_ink = self.frame_ink(k, i)
            # approximate the likelihood from the GP field generated by the N_nbr_max nearest observations
            # the attention is applied here by only considering maximum N_nbr_num nearest neighbors
            n_nbr = np.min([self.Util.N_nbr_num, len(frame_ink.x)])
            points = np.vstack((frame_ink.x, frame_ink.y)).T
            knn = NearestNeighbors(n_neighbors=n_nbr, p=1)
            knn.fit(points)
            querry_point = np.array([self.frames[i].x, self.frames[i].y]).T
            n_idx = knn.kneighbors(querry_point, return_distance=False)
            n_idx = np.unique(n_idx)
            near_frame = Frame(frame_ink.x[n_idx], frame_ink.y[n_idx], frame_ink.vx[n_idx], frame_ink.vy[n_idx])
            frame_i = self.frames[i]
            ux_pos, uy_pos, covx_pos, covy_pos, log_likelihood = self.b[k].GP_posterior(frame_i, near_frame)
            # if likelihood == 0:
            #     print('frame{} in pattern{} GP post frame_ink likel: Returning -inf'.format(i, k))
            #     return float("-inf")
            # else:
            if True:
                # print('frame{} in pattern{} GP post frame_ink likel: {}'.format(i, k, log_likelihood))
                return log_likelihood

    def frame_ink(self, k, i, all_frame=False):
        # frame ink stores all the data in kth pattern except the ith frame
        idx = np.where(np.array(self.z) == k)
        idx = np.asarray(idx)
        Idx = idx[0].astype(int)
        # check whether i exit
        if not all_frame:
            Idx = Idx[Idx != i]
        frame_list = []
        for idx_iter in range(len(Idx)):
            frame_list.append(self.frames[Idx[idx_iter]])
        # get all concatenation
        frames_i = self.combined_frame(frame_list)
        return frames_i

    def show_mixture_model(self):
        print('alpha: ', self.alpha, ' n: ', self.n, ' K: ', self.K)
        print('assignmentIter: ', self.assignmentIteration, ' paraIter: ', self.parameterIteration)
        print('----------------------------')

    def cov_mean(self, frames):
        # input: frames
        # output: mean and covariance of the combined frame
        combinedframes = self.combined_frame(frames)
        ux = np.mean(combinedframes.vx)
        uy = np.mean(combinedframes.vy)
        sigmax = np.std(combinedframes.vx)
        sigmay = np.std(combinedframes.vy)
        return ux, uy, sigmax, sigmay, self.Util.sigman

    @staticmethod
    def z2partition(z, k):
        # convert assignment z to partition
        partition = [0] * k
        for i in range(len(z)):
            partition[z[i]] += 1
        return partition

    @staticmethod
    def combined_frame(frames):
        # combine all input frames to a single unified frame
        x_ink = y_ink = vx_ink = vy_ink = []
        if len(frames) == 1:
            return frames[0]
        else:
            for i in range(len(frames)):
                x_ink = np.concatenate((x_ink, frames[i].x), axis=0)
                y_ink = np.concatenate((y_ink, frames[i].y), axis=0)
                vx_ink = np.concatenate((vx_ink, frames[i].vx), axis=0)
                vy_ink = np.concatenate((vy_ink, frames[i].vy), axis=0)
            return Frame(x_ink, y_ink, vx_ink, vy_ink)