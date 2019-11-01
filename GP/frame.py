#!usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import scipy
from scipy.stats import gamma
from util import Util



class Frame(object):
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.Util = Util()

    def plot_frame(self):
        plt.scatter(self.x, self.y, color='k')
        plt.quiver(self.x, self.y, self.vx, self.vy,color='b')
        plt.xlim([self.Util.roi_xMin, self.Util.roi_xMax])
        plt.ylim([self.Util.roi_yMin, self.Util.roi_yMax])
        plt.show()

    @staticmethod
    def animate_frames(frames,delT=0.1):
        u = Util()
        plt.ion()
        for i in range(len(frames)):
            plt.cla()
            frame_temp = frames[i]
            # frame_temp.plot_frame()
            plt.scatter(frame_temp.x, frame_temp.y, color='k')
            plt.quiver(frame_temp.x, frame_temp.y, frame_temp.vx, frame_temp.vy, color='b',scale=100)
            plt.xlim([u.roi_xMin, u.roi_xMax])
            plt.ylim([u.roi_yMin, u.roi_yMax])

            plt.title('Frame id: {}'.format(i))
            plt.show()
            plt.pause(delT)
        plt.ioff()

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

