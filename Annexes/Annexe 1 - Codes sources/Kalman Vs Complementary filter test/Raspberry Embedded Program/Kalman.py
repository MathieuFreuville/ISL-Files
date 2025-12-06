

import time

class KFilter:

    Q_angle = 0.001         # Process noise variance for the accelerometer
    Q_bias = 0.003          # Process noise variance for the gyro bias
    R_measure = 0.03        # Measurement noise variance - this is actually the variance of the measurement noise

    '''
    Note that if you set the measurement noise variance var(v) too high the filter will respond really slowly as it is trusting new measurements less,
    but if it is too small the value might overshoot and be noisy since we trust the accelerometer measurements too much.
    '''
    Angle = 0.0             # The angle calculated by the Kalman filter - part of the 2x1 state vector
    Bias = 0.0              # The gyro bias calculated by the Kalman filter - part of the 2x1 state vector

    P = [[0.0 , 0.0],[0.0 , 0.0]]

    def __init__(self, angle = 0.0, q_angle = 0.001, q_bias = 0.003, r_measure = 0.03):
        self.Q_angle = q_angle
        self.Q_bias = q_bias
        self.R_measure = r_measure

        self.Angle = angle                          # The angle calculated by the Kalman filter - part of the 2x1 state vector
        self.Bias = 0.0                             # Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

        self.P = [[0.0 , 0.0],[0.0 , 0.0]]          # Error covariance matrix - This is a 2x2 matrix


    def calculateangle(self, newangle, newrate, dt):

        #global Angle
        #global Bias
        #global P

        #-------
        # Step 1
        #-------
        Rate = newrate - self.Bias
        self.Angle += dt * Rate


        #-------
        # Step 2
        #-------
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        #-------
        # Step 3 (noted step 4)
        #-------
        S = self.P[0][0] + self.R_measure

        #-------
        # Step 4 (noted step 5)
        #-------
        K = [0.0 , 0.0]
        K[0] = self.P[0][0] / S
        K[1] = self.P[1][0] / S


        #-------
        # Step 5 (noted step3)
        #-------
        y = newangle - self.Angle


        #-------
        # Step 6
        #-------
        self.Angle += K[0] * y
        self.Bias += K[1] * y

        #-------
        # Step 7
        #-------
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.Angle