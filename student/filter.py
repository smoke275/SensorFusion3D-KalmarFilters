# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state # process model dimension
        self.dt = params.dt # time increment
        self.q = params.q # process noise variable for Kalman filter Q

    def F(self):
        dt = self.dt
        return np.matrix([[1, 0, 0, dt, 0,  0 ],
                          [0, 1, 0, 0,  dt, 0 ],
                          [0, 0, 1, 0,  0,  dt],
                          [0, 0, 0, 1,  0,  0 ],
                          [0, 0, 0, 0,  1,  0 ],
                          [0, 0, 0, 0,  0,  1 ]])
        

    def Q(self):
        q = self.q
        dt = self.dt
        q1 = ((dt**3)/3) * q 
        q2 = ((dt**2)/2) * q 
        q3 = dt * q    
        return np.matrix([[q1, 0,  0,  q2, 0,  0 ],
                          [0,  q1, 0,  0,  q2, 0 ],
                          [0,  0,  q1, 0,  0,  q2],
                          [q2, 0,  0,  q3, 0,  0 ],
                          [0,  q2, 0,  0,  q3, 0 ],
                          [0,  0,  q2, 0,  0,  q3]])
        

    def predict(self, track):
        F = self.F()
        x = F * track.x # state prediction
        P = F * track.P * F.transpose() + self.Q() # covariance prediction
        track.set_x(x)
        track.set_P(P)
        

    def update(self, track, meas):
        x = track.x
        P = track.P
        H = meas.sensor.get_H(x) # measurement matrix
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H) # covariance of residual
        K = P * H.transpose() * np.linalg.inv(S) # Kalman gain
        x = x + K * gamma # state update
        I = np.identity(self.dim_state)
        P = (I - K * H) * P # covariance update
        track.set_x(x)
        track.set_P(P)
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        x = track.x
        H = meas.sensor.get_H(x) # measurement matrix
        return meas.z - H * x # residual
        

    def S(self, track, meas, H):
        return H * track.P * H.transpose() + meas.R # covariance of residual