#!usr/bin/env python3

import matplotlib.pyplot as plt
import scipy
import math
import numpy as np
from scipy.stats import gamma, multivariate_normal
from numpy.linalg import inv

from util import Util
from frame import Frame

# MotionPattern is a class with functions to
# a) initialize motion pattern represented by Gaussian Process
# b) update motion pattern parameters ux, uy, wx and wy


class MotionPattern(object):
    def __init__(self, ux=0.0, uy=0.0, sigmax=1.0, sigmay=1.0, sigman=1.0, wx=1.0, wy=1.0):
        self.ux = ux
        self.uy = uy
        self.sigmax = sigmax
        self.sigmay = sigmay
        self.sigman = sigman
        self.wx = wx
        self.wy = wy
        self.Util = Util()

    def update_para(self, frames):
        # search the best wx,wy parameters for its assigned frames and
        # return the motion pattern with the updated parameters
        if not self.Util.useMLE:
            try:
                self.update_para_sample(frames)
            except:
                wx, wy, pwx, pwy = self.Util.draw_w()
                self.wx = wx
                self.wy = wy
        else:
            self.update_para_MLE(frames)
        return self

    def update_para_sample(self, frames):
        # update kernel parameters wx and wy
        x = np.linspace(1, 51, 51)
        [WX,WY] = np.meshgrid(x, x)
        WX = np.reshape(WX, (-1, 1))
        WY = np.reshape(WY, (-1, 1))
        # prior
        PWX = gamma.pdf(WX, a=self.Util.gammaShape, scale=self.Util.gammaScale)
        PWY = gamma.pdf(WY, a=self.Util.gammaShape, scale=self.Util.gammaScale)
        log_PWXWY_prior = np.log(np.multiply(PWX, PWY))
        # likelihood
        log_PWXWY_likelihood = np.zeros(len(log_PWXWY_prior))
        for i in range(len(WX)):
            self.wx = WX[i]
            self.wy = WY[i]
            log_PWXWY_likelihood[i] = np.log(self.GP_prior(frames))
        # posterior
        log_PWXWY_post = log_PWXWY_prior + log_PWXWY_likelihood
        log_PWXWY_post = log_PWXWY_post - max(log_PWXWY_post) # normalization
        PWXWY_post = np.exp(log_PWXWY_post)
        # resample based on posterior prob
        candidate = np.linspace(0, len(PWXWY_post)-1, len(PWXWY_post))
        idx = np.random.choice(candidate, 1, PWXWY_post)
        # uptate wx wy
        self.wy = WY[int(idx)]
        self.wx = WX[int(idx)]
        # print(self.wx)

    def update_para_MLE(self, frames):
        # not used in algorithm, will be implemented later
        pass

    def squared_exp_cov(self, x1, y1, x2, y2, bnoise):
        # calculate the covariance matrix given location data and motion pattern.
        # kernel function:
        # k(x, x*) = sigmax^2*exp(-(x - x*)^2/(2*wx^2) - (y - y*)^2/(2*wy^2))
        # k(y, y*) = sigmay^2*exp(-(x - x*)^2/(2*wx^2) - (y - y*)^2/(2*wy^2))
        # input: x1,y1 in R^(nx1), x2,y2 in R(mx1)
        # output: xK in R^(nxn), yK in R(nxn) are both PSD

        X2, X1 = np.meshgrid(x2, x1)
        Y2, Y1 = np.meshgrid(y2, y1)

        disMat = -(X1-X2)**2/(2*self.wx**2) - (Y1-Y2)**2/(2*self.wy**2)
        if bnoise:
            xK = self.sigmax**2 * np.exp(disMat) + self.sigman**2 * np.eye(len(x1))
            yK = self.sigmay**2 * np.exp(disMat) + self.sigman**2 * np.eye(len(x1))
        else:
            xK = self.sigmax**2 * np.exp(disMat)
            yK = self.sigmay**2 * np.exp(disMat)
        return xK, yK

    def GP_posterior(self, frame_test, frame_train, prediction=False):
        # calculate the likelihood of a frame under motion pattern with
        # given data.
        # x,y: frame testing (with *)
        # X,Y: frame training (no *)
        m = 0
        xKXYXY, yKXYXY = self.squared_exp_cov(frame_train.x, frame_train.y, frame_train.x, frame_train.y, True)
        xKxyxy, yKxyxy = self.squared_exp_cov(frame_test.x, frame_test.y, frame_test.x, frame_test.y, False)
        xKxyXY, yKxyXY = self.squared_exp_cov(frame_test.x, frame_test.y, frame_train.x, frame_train.y, False)
        xKXYxy = np.transpose(xKxyXY)
        yKXYxy = np.transpose(yKxyXY)

        xtemp = np.dot(xKxyXY, inv(xKXYXY))
        ytemp = np.dot(yKxyXY, inv(yKXYXY))
        ux_pos = m*self.ux * np.ones_like(frame_test.x) + np.dot(xtemp, (frame_train.vx - m*self.ux*np.ones_like(frame_train.x)))
        uy_pos = m*self.uy * np.ones_like(frame_test.y) + np.dot(ytemp, (frame_train.vy - m*self.uy*np.ones_like(frame_train.y)))

        covx_pos = xKxyxy - np.dot(xtemp, xKXYxy)
        covy_pos = yKxyxy - np.dot(ytemp, yKXYxy)

        covx_pos = (covx_pos + np.transpose(covx_pos)) / 2.0 + \
                   self.Util.eip_post * np.eye(covx_pos.shape[0], covx_pos.shape[1])
        covy_pos = (covy_pos + np.transpose(covy_pos)) / 2.0 + \
                   self.Util.eip_post * np.eye(covy_pos.shape[0], covy_pos.shape[1])
        # eig1 = np.linalg.det(covx_pos)
        # eig2 = np.linalg.det(covy_pos)
        s1, v = scipy.linalg.eigh(covx_pos)
        s2, v = scipy.linalg.eigh(covy_pos)
        if prediction:
            return ux_pos, uy_pos, covx_pos, covy_pos
        else:
            if min(s1) < -np.finfo(float).eps or min(s2) < -np.finfo(float).eps:
                print('covariance matrix should be PSD')
                likelihood = 0
                return ux_pos, uy_pos, covx_pos, covy_pos, likelihood
            else:
                # print('yeah not singular cov_post')
                # temp1 = self.norm_pdf_multivariate(frame_test.vx, ux_pos, covx_pos)
                # temp2 = self.norm_pdf_multivariate(frame_test.vy, uy_pos, covy_pos)
                temp1 = multivariate_normal.pdf(frame_test.vx, ux_pos, covx_pos)
                temp2 = multivariate_normal.pdf(frame_test.vy, uy_pos, covy_pos)
                # print('GP posterior likel temp1:{} temp2:{} temp1_lib:{} temp2_lib:{}'.format(temp1, temp2, temp1_lib, temp2_lib))
                likelihood = temp1 * temp2
                np.seterr(divide='ignore')
                log_likelihod = np.log(temp1) + np.log(temp2)
                return ux_pos, uy_pos, covx_pos, covy_pos, log_likelihod

    def norm_pdf_multivariate(self, x, mu, sigma):
        # Self implemented multivariate normal distribution
        # input: x, query array; mu: mean; sigma: PSD covariance function
        # output: prob of drawing x from the distribution

        size = len(x)
        if size == len(mu) and (size, size) == sigma.shape:
            det = np.linalg.det(sigma)
            # print('det', det)
            if det == 0:
                raise NameError("The covariance matrix can't be singular")

            norm_const = 1.0 / (math.pow((2 * np.pi), size / 2.0) * math.pow(det, 0.5))
            x_mu = np.array(x - mu)
            inv = np.linalg.inv(sigma)
            inner = np.dot(x_mu, inv)
            outer = np.dot(inner, np.transpose(x_mu))
            result = math.pow(math.e, -0.5 * outer)
            final = norm_const * result
            return final
        else:
            raise NameError("The dimensions of the input don't match")

    def GP_prior(self, framesTest):
        # calculate the likelihood of a testing frame under a GP
        # without observing any data

        covx, covy = self.squared_exp_cov(framesTest.x, framesTest.y, framesTest.x, framesTest.y, False)
        covx = (covx + np.transpose(covx)) / 2.0 + self.Util.eip_prior * np.eye(covx.shape[0], covx.shape[1])
        covy = (covy + np.transpose(covy)) / 2.0 + self.Util.eip_prior * np.eye(covy.shape[0], covy.shape[1])
        ux_prior = self.ux * np.ones_like(framesTest.vx)
        uy_prior = self.uy * np.ones_like(framesTest.vy)
        # eig1 = np.linalg.det(covx)
        # eig2 = np.linalg.det(covy)
        s1, v = scipy.linalg.eigh(covx)
        s2, v = scipy.linalg.eigh(covy)
        if min(s1) < -np.finfo(float).eps or min(s2) < -np.finfo(float).eps:
            print('covariance matrix should be PSD')
            likelihood = 0
            return likelihood
        else:
            # print('yeah not singular cov_prior')
            # temp1 = multivariate_normal.pdf(framesTest.vx, ux_prior, covx)
            # temp2 = multivariate_normal.pdf(framesTest.vy, uy_prior, covy)
            temp1 = self.norm_pdf_multivariate(framesTest.vx, ux_prior, covx)
            temp2 = self.norm_pdf_multivariate(framesTest.vy, uy_prior, covy)
            likelihood = temp1*temp2
            return likelihood

    def plotPattern(self,frameTrain=0,frameTest=0,bplotTrainingFrame = False, pattern_num = -1): 
        handles = [] # handling legends
        
        # If no frameTest, generate grid frame and visualize pattern as a field
        if frameTest == 0:
            X_roi = np.linspace(self.Util.roi_xMin,self.Util.roi_xMax,num = 20,endpoint = True).T
            Y_roi = np.linspace(self.Util.roi_yMin,self.Util.roi_yMax,num = 20,endpoint = True).T
            X,Y = np.meshgrid(X_roi,Y_roi)
            X = X.reshape(-1,1).ravel()
            Y = Y.reshape(-1,1).ravel()

            frameTest = Frame(X,Y,np.zeros(X.shape),np.zeros((Y.shape))) # Grid frame
        else:
            frameTest_veh = plt.scatter(frameTest.x, frameTest.y, color='r', s = 100, label='Test frame vehicle')
            handles.append(frameTest_veh)
            
        ux_prior = self.ux * np.ones_like(frameTest.vx)
        uy_prior = self.uy * np.ones_like(frameTest.vy)
        
        # If no training data, plot prior of pattern
        if frameTrain == 0:
            bplotTrainingFrame = False
            ux_plot = ux_prior
            uy_plot = uy_prior
        else: # Calculate posterior velocity given training data
            ux_pos,uy_pos,_,_,_ = self.GP_posterior(frameTest,frameTrain)
            ux_plot = ux_pos
            uy_plot = uy_pos
        
        frameTest_quiv = plt.quiver(frameTest.x, frameTest.y, ux_plot, uy_plot, color='g',scale =200, label = 'Mean field')
        handles.append(frameTest_quiv)

        # Plot training data for the pattern
        if bplotTrainingFrame == True: 
            frameTrain_veh = plt.scatter(frameTrain.x, frameTrain.y, color='k', s = 100, label = 'Training frame vehicle')
            handles.append(frameTrain_veh)
            frameTrain_quiv = plt.quiver(frameTrain.x, frameTrain.y, frameTrain.vx, frameTrain.vy, color='b',scale =200,label = 'Training frame vehicle velocity')
            handles.append(frameTrain_quiv)

        plt.xlim([self.Util.roi_xMin, self.Util.roi_xMax])
        plt.ylim([self.Util.roi_yMin, self.Util.roi_yMax])
        plt.legend(handles = handles)
        # plt.savefig('Ped_zara_results/zara_pattern{}.png'.format(pattern_num))
        if pattern_num != -1:
            plt.title('Pattern {}'.format(pattern_num))
        # plt.close()
        plt.show()

    
    def updateFramebyPattern(self,frameTrain,frameOld):
        ux_pos,uy_pos,_,_,_ = self.GP_posterior(frameOld,frameTrain)
        dt = self.Util.delT_data
        frameNew = Frame(frameOld.x + dt*ux_pos, frameOld.y + dt*uy_pos, ux_pos, uy_pos)
        return frameNew




