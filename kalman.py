
import numpy as np
import random
import matplotlib.pyplot as plt

class KalmanFilter:
    '''
    Inspired by http://greg.czerniak.info/guides/kalman1/
    '''
    def __init__(self, _A, _B, _Q, _H, _R, xhat, P):
        self.A = _A
        self.B = _B
        self.Q = _Q
        self.H = _H
        self.R = _R
        self.state_estimate = xhat
        self.prob_estimate = P

    def step(self, control_vector, measurement_vector):
        state_predicted = self.A * self.state_estimate + self.B * control_vector
        covariance_predicted = (self.A * self.prob_estimate) * np.transpose(self.A) + self.Q
        innovation = measurement_vector - self.H * state_predicted
        innovation_covariance = (self.H * covariance_predicted) * np.transpose(self.H) + self.R
        gain = covariance_predicted * np.transpose(self.H) * np.linalg.inv(innovation_covariance)
        self.state_estimate = state_predicted + gain * innovation
        size = self.prob_estimate.shape[0]
        self.prob_estimate = (np.eye(size) - gain * self.H) * covariance_predicted

    def get_state(self):
        return self.state_estimate


trg_temp = 72.0
sensed_bias = 0.0
sensed_sigma = 3.5

# State transition matrix. Multiply state by this and add control factors
# and you get a prediction of the state for the next time step
A = np.matrix([1.0])

# Control Matrix, the amount of control we have over the model
B = np.matrix([0.0])

# Observation matrix. Multiply a state vector by H to translate it to a measurement vector.
H = np.matrix([1.0])

# Process covariance. This should represent the amount of covariance expected
# in the simulated system.  IE this should represent how much covariance is
# captured in the state transition (A) and control matrices (B).
Q = np.matrix([0.00001])

# Measurement covariance.
R = np.matrix([0.1])

# Initial state estimate
xhat = np.matrix([60])

# Initial covariance estimate (garbage)
P = np.matrix([1])

filt = KalmanFilter(A, B, Q, H, R, xhat, P)



data = []
for ts in np.arange(0, 300):
    measured_temp = random.gauss(trg_temp + sensed_bias, sensed_sigma)
    estimated_temp = filt.get_state()
    data.append((ts, trg_temp, measured_temp, estimated_temp[0,0]))
    filt.step(np.matrix([0]), np.matrix([measured_temp]))

ts, trg, meas_temp, est_temp = zip(*data)
plt.subplot(1, 1, 1)
plt.xlabel('time (s)')
plt.title('Kalman Test')
plt.ylabel('Temp')
plt.plot(ts, meas_temp, 'bo')
plt.plot(ts, trg, 'b-')
plt.plot(ts, est_temp, 'g-')
plt.show()


