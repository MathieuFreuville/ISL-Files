
import time
from matplotlib import pyplot as plt
from IMU import MPU6050
from Kalman import KFilter

class MPUFilterTest:

    Timer = pow(10,9)                   # Timer in nanoseconds, set by default to 1s
    SimTime = 10                        # Simulation time in seconds
    Samples = 0
    sensor = None
    sampling_frequency= None

    def __init__(self, timer = pow(10,9), simulation_time = 10):

        self.Timer = timer
        self.SimTime = simulation_time
        self.Samples =   simulation_time * pow(10,9) / timer
        self.sampling_frequency = pow(10,9) / timer

    def initialize_sensor(self, comp_filt = False):

        self.sensor = MPU6050(address=0x68, compl_filter = comp_filt)
        if comp_filt:
            self.sensor.set_complementaryfilter_ratios(gyro = 0.99)
        self.sensor.calibrate_gyros()
        self.sensor.calibrate_acceleros()
        self.sensor.set_accel_range(self.sensor.ACCEL_RANGE_2G)
        self.sensor.set_gyro_range(self.sensor.GYRO_RANGE_250DEG)
        self.sensor.set_filter_range(self.sensor.FILTER_BW_256)
        self.sensor.initialize_gyros_angles()

    def simpleplot(self, t_axis, x_axis, y_axis, z_axis, graph_title = ''):

        my_legend = []
        if not x_axis == 0:
            plt.plot(t_axis, x_axis, color='red')
            my_legend.append('X Axis')
            # plt.ylim(-2.5,2.5)
        if not y_axis == 0:
            plt.plot(t_axis, y_axis, color = 'green')
            my_legend.append('Y Axis')
        if not z_axis == 0:
            plt.plot(t_axis, z_axis, color='blue')
            my_legend.append('Z Axis')

        plt.legend(my_legend)
        plt.xlabel('Time (s)')
        plt.ylabel('Angle displacement (°)')
        plt.title(graph_title)
        plt.grid()
        plt.show()
        plt.close()

    def dualplot(self, t_axis, y0_axis, y1_axis, graph_title = '',title1 = '',title2 = '', legend = ''):

        # Plotting result Gyro Vs complementary filter X
        fig, axs = plt.subplots(2)
        fig.suptitle(graph_title)
        axs[0].plot(t_axis, y0_axis)
        axs[0].legend([legend])
        axs[1].plot(t_axis, y1_axis)
        axs[1].legend([legend])
        axs[0].set_title(title1)
        axs[1].set_title(title2)

        for ax in axs.flat:
            ax.set(xlabel='time (s)', ylabel='Angle displacement (°)')
            ax.grid()
        # Hide x labels and tick labels for top plots and y ticks for right plots.
        for ax in axs.flat:
            ax.label_outer()
        plt.show()

    def test_acceleros(self):

        self.initialize_sensor(comp_filt=False)
        print('Entering in accelerometers test sequence. Sampling frequency = ' + str(self.sampling_frequency) + ' Hz - Test duration = ' + str(self.SimTime) + ' seconds')

        i = 0
        angles = self.sensor.get_accel_angles()



        dt = self.Timer / pow(10,9)

        # Declaration of the arrays for storing gyro values  for  graph
        t = [0]  # Used for plotting graph about Gyro offset
        X = [angles['x']]  # Used for plotting graph about Gyro offset
        Y = [angles['y']]  # Used for plotting graph about Gyro offset
        Z = [angles['z']]  # Used for plotting graph about Gyro offset


        while i < self.Samples:

            t0 = time.clock_gettime_ns(time.CLOCK_REALTIME)
            acc_data = self.sensor.get_accel_angles()

            t.append(t[-1] + dt)
            X.append(acc_data['x'])
            Y.append(acc_data['y'])
            Z.append(acc_data['z'])

            i = i + 1

            # When the loop is finished, wait until the full time is Elapsed
            dt = 0
            while dt < self.Timer:
                t1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                dt = t1 - t0  # duration is integer with nanosecond resolution
            dt = dt / pow(10,9)

        self.simpleplot(t, X, 0, 0, ('MPU6050 accelerometer over time \n Sampling Frequency = '
                                     + str(self.sampling_frequency) + 'Hz - No filtering \n'
                                     'Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz'))

        self.simpleplot(t, 0, Y, 0, ('MPU6050 accelerometer over time \n Sampling Frequency = '
                                     + str(self.sampling_frequency) + 'Hz - No filtering \n'
                                     'Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz'))

        self.simpleplot(t, 0, 0, Z, ('MPU6050 accelerometer over time \n Sampling Frequency = '
                                     + str(self.sampling_frequency) + 'Hz - No filtering \n'
                                     'Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz'))

    def test_lowpass_filter(self):

        self.initialize_sensor(comp_filt=False)
        print('Entering in accelerometers test sequence. Sampling frequency = ' + str(self.sampling_frequency) + ' Hz - Test duration = ' + str(self.SimTime) + ' seconds')

        switch_filter = 0
        i = 0
        angles = self.sensor.get_accel_angles()

        dt = self.Timer / pow(10,9)

        # Declaration of the arrays for storing gyro values  for  graph
        t = [0]  # Used for plotting graph about Gyro offset
        X = [angles['x']]  # Used for plotting graph about Gyro offset
        Y = [angles['y']]  # Used for plotting graph about Gyro offset
        Z = [angles['z']]  # Used for plotting graph about Gyro offset

        while switch_filter < 6:
            if switch_filter == 0: self.sensor.set_filter_range(self.sensor.FILTER_BW_256)
            elif switch_filter == 1: self.sensor.set_filter_range(self.sensor.FILTER_BW_188)
            elif switch_filter == 2: self.sensor.set_filter_range(self.sensor.FILTER_BW_98)
            elif switch_filter == 3: self.sensor.set_filter_range(self.sensor.FILTER_BW_42)
            elif switch_filter == 4: self.sensor.set_filter_range(self.sensor.FILTER_BW_20)
            elif switch_filter == 5: self.sensor.set_filter_range(self.sensor.FILTER_BW_5)
            switch_filter += 1

            i = 0

            while i < self.Samples:

                t0 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                acc_data = self.sensor.get_accel_angles()

                t.append(t[-1] + dt)
                X.append(acc_data['x'])
                Y.append(acc_data['y'])
                Z.append(acc_data['z'])

                i = i + 1

                # When the loop is finished, wait until the full time is Elapsed
                dt = 0
                while dt < self.Timer:
                    t1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                    dt = t1 - t0  # duration is integer with nanosecond resolution
                dt = dt / pow(10,9)

        self.simpleplot(t, X, 0, 0, ('MPU6050 accelerometer over time \n Sampling Frequency = '
                                     + str(self.sampling_frequency) + 'Hz - No software filtering \n'
                                     'Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW reduced from 260 to 5 Hz every ' + str(self.SimTime) + ' seconds'))

        self.simpleplot(t, 0, Y, 0, ('MPU6050 accelerometer over time \n Sampling Frequency = '
                                     + str(self.sampling_frequency) + 'Hz - No software filtering \n'
                                     'Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW reduced from 260 to 5 Hz every ' + str(self.SimTime) + ' seconds'))

        self.simpleplot(t, 0, 0, Z, ('MPU6050 accelerometer over time \n Sampling Frequency = '
                                     + str(self.sampling_frequency) + 'Hz - No software filtering \n'
                                     'Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW reduced from 260 to 5 Hz every ' + str(self.SimTime) + ' seconds'))


    def test_gyro_drift(self, Xinit=0, Yinit=0, Zinit=0):

        self.initialize_sensor(comp_filt=False)
        print('Entering in gyro drift test sequence. Sampling frequency = ' + str(self.sampling_frequency) + ' Hz - Test duration = ' + str(self.SimTime) + ' seconds')

        i = 0

        angle_x = Xinit  # Total travelled angle X
        angle_y = Yinit  # Total travelled angle Y
        angle_z = Zinit  # Total travelled angle Z

        dt = self.Timer / pow(10,9)

        # Declaration of the arrays for storing gyro values  for  graph
        t = [0]  # Used for plotting graph about Gyro offset
        X = [angle_x]  # Used for plotting graph about Gyro offset
        Y = [angle_y]  # Used for plotting graph about Gyro offset
        Z = [angle_z]  # Used for plotting graph about Gyro offset


        while i < self.Samples:

            t0 = time.clock_gettime_ns(time.CLOCK_REALTIME)
            Gyrdata = self.sensor.get_gyro_angular_displacement(dt)

            t.append(t[-1] + dt)
            X.append(X[-1] + (Gyrdata['x']))
            Y.append(Y[-1] + (Gyrdata['y']))
            Z.append(Z[-1] + (Gyrdata['z']))

            i = i + 1

            # When the loop is finished, wait until the full time is Elapsed
            dt = 0
            while dt < self.Timer:
                t1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                dt = t1 - t0  # duration is integer with nanosecond resolution
            dt = dt / pow(10,9)

        self.simpleplot(t, X, Y, Z, ('MPU6050 Gyro drift over time \n Sampling Frequency = '
                                     + str(self.sampling_frequency) + 'Hz - No software filtering \n'
                                     'Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz'))

    def test_gyro_drift_complementary(self, x_init=0, y_init=0, z_init=0):

        self.initialize_sensor(comp_filt=True)

        print('Entering in gyro drift test sequence. Sampling frequency = ' + str(self.sampling_frequency)
              + ' Hz - Test duration = ' + str(self.SimTime) + ' seconds' + 'Gyro = ' + str(self.sensor.Comp_Gyr_factor)
              + ' Acc: ' + str(self.sensor.Comp_Acc_factor))

        i = 0
        dt = self.Timer / pow(10, 9)  # Assume dt is correct for the first sample

        initial_angles = self.sensor.get_accel_angles()

        # Declaration of the arrays for storing gyro values  for  graph
        t = [0]  # Used for plotting graph about Gyro offset
        X = [initial_angles['x']]  # Used for plotting graph about Gyro offset
        Y = [initial_angles['y']]  # Used for plotting graph about Gyro offset
        Z = [initial_angles['z']]  # Used for plotting graph about Gyro offset
        Xcomp = [initial_angles['x']]                    # Used for plotting graph about Gyro offset
        Ycomp = [initial_angles['y']]                    # Used for plotting graph about Gyro offset
        Zcomp = [initial_angles['z']]                    # Used for plotting graph about Gyro offset

        while i < self.Samples:

            t0 = time.clock_gettime_ns(time.CLOCK_REALTIME)

            data = self.sensor.get_6axis(dt)

            t.append( t[-1] + dt)
            X.append(self.sensor.Gyr_Angle_X)
            Y.append(self.sensor.Gyr_Angle_Y)
            Z.append(self.sensor.Gyr_Angle_Z)
            Xcomp.append(self.sensor.Comp_Angle_X)
            Ycomp.append(self.sensor.Comp_Angle_Y)
            Zcomp.append(self.sensor.Comp_Angle_Z)

            i = i + 1

            # When the loop is finished, wait until the full time is Elapsed
            dt = 0
            while dt < self.Timer:
                t1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                dt = t1 - t0  # duration is integer with nanosecond resolution
            dt = dt / pow(10, 9)

        self.simpleplot(t, Xcomp, Ycomp, Zcomp, ('MPU6050 Gyro drift over time \n Sampling Frequency = '
            + str(self.sampling_frequency) + 'Hz - Complementary Filter Gyro =' + str(self.sensor.Comp_Gyr_factor) + ' Acc = ' + str(self.sensor.Comp_Acc_factor) + '\n'
            'MPU6050 Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz'))

    def compare_gyro_drift_complementary(self, x_init=0, y_init=0, z_init=0):

        self.initialize_sensor(comp_filt=True)
        print('Entering in gyro drift comparison test sequence. Sampling frequency = ' + str(self.sampling_frequency)
              + ' Hz - Test duration = ' + str(self.SimTime) + ' seconds' + 'Gyro = ' + str(self.sensor.Comp_Gyr_factor)
              + ' Acc: ' + str(self.sensor.Comp_Acc_factor))

        i = 0
        dt = self.Timer / pow(10, 9)  # Assume dt is correct for the first sample

        angles = self.sensor.get_accel_angles()
        # Declaration of the arrays for storing gyro values  for  graph
        t = [0]  # Used for plotting graph about Gyro offset
        X = [self.sensor.Comp_Angle_X]  # Used for plotting graph about Gyro offset
        Y = [self.sensor.Comp_Angle_Y]  # Used for plotting graph about Gyro offset
        Z = [self.sensor.Comp_Angle_Z]  # Used for plotting graph about Gyro offset
        Xcomp = [self.sensor.Comp_Angle_X]  # Used for plotting graph about Gyro offset
        Ycomp = [self.sensor.Comp_Angle_Y]  # Used for plotting graph about Gyro offset
        Zcomp = [self.sensor.Comp_Angle_Z]  # Used for plotting graph about Gyro offset



        while i < self.Samples:

            t0 = time.clock_gettime_ns(time.CLOCK_REALTIME)

            data = self.sensor.get_6axis(dt)

            t.append(t[-1] + dt)
            X.append(self.sensor.Gyr_Angle_X)
            Y.append(self.sensor.Gyr_Angle_Y)
            Z.append(self.sensor.Gyr_Angle_Z)
            Xcomp.append(self.sensor.Comp_Angle_X)
            Ycomp.append(self.sensor.Comp_Angle_Y)
            Zcomp.append(self.sensor.Comp_Angle_Z)

            i = i + 1

            # When the loop is finished, wait until the full time is Elapsed
            dt = 0
            while dt < self.Timer:
                t1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                dt = t1 - t0  # duration is integer with nanosecond resolution
            dt = dt / pow(10, 9)

        long_title = ('MPU6050 Gyro drift over time \n Sampling Frequency = '
            + str(self.sampling_frequency) + 'Hz  +  MPU6050 Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz')

        self.dualplot(t, X, Xcomp, long_title, '','Complementary Filter Gyro = ' +
                      str(self.sensor.Comp_Gyr_factor) + ' Acc = ' + str(self.sensor.Comp_Acc_factor), 'X axis')


        long_title = ('MPU6050 Gyro drift over time \n Sampling Frequency = '
            + str(self.sampling_frequency) + 'Hz  +  MPU6050 Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz')

        self.dualplot(t, Y, Ycomp, long_title, '','Complementary Filter Gyro = ' +
                      str(self.sensor.Comp_Gyr_factor) + ' Acc = ' + str(self.sensor.Comp_Acc_factor), 'Y axis' )

        long_title = ('MPU6050 Gyro drift over time \n Sampling Frequency = '
            + str(self.sampling_frequency) + 'Hz  +  MPU6050 Acc Range = ' + str(self.sensor.read_accel_range()) + ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz')

        self.dualplot(t, Z, Zcomp, long_title, '','Complementary Filter Gyro = ' +
                      str(self.sensor.Comp_Gyr_factor) + ' Acc = ' + str(self.sensor.Comp_Acc_factor), 'Z axis')

    def compare_all_filters(self, x_init=0, y_init=0, z_init=0):

        self.initialize_sensor(comp_filt=True)
        print('Entering in filters. Sampling frequency = ' + str(self.sampling_frequency)
              + ' Hz - Test duration = ' + str(self.SimTime) + ' seconds' + 'Gyro = ' + str(self.sensor.Comp_Gyr_factor)
              + ' Acc: ' + str(self.sensor.Comp_Acc_factor))

        i = 0
        dt = self.Timer / pow(10, 9)  # Assume dt is correct for the first sample

        kalman_Filter_1 = KFilter(self.sensor.Comp_Angle_X, q_angle = 0.001, q_bias = 0.003, r_measure = 0.05)
        kalman_Filter_2 = KFilter(self.sensor.Comp_Angle_X, q_angle = 0.001, q_bias = 0.003, r_measure = 0.03)
        kalman_Filter_3 = KFilter(self.sensor.Comp_Angle_X, q_angle = 0.001, q_bias = 0.003, r_measure = 0.01)

        # Declaration of the arrays for storing gyro values  for  graph
        t = [0]  # Used for plotting graph about Gyro offset
        X = [self.sensor.Comp_Angle_X]  # Used for plotting graph about Gyro offset
        Y = [self.sensor.Comp_Angle_Y]  # Used for plotting graph about Gyro offset
        Z = [self.sensor.Comp_Angle_Z]  # Used for plotting graph about Gyro offset
        Xcomp = [self.sensor.Comp_Angle_X]  # Used for plotting graph about Gyro offset
        Ycomp = [self.sensor.Comp_Angle_Y]  # Used for plotting graph about Gyro offset
        Zcomp = [self.sensor.Comp_Angle_Z]  # Used for plotting graph about Gyro offset
        K1 = [self.sensor.Comp_Angle_X]  # Used for plotting graph about Kalman Filter 1
        K2 = [self.sensor.Comp_Angle_X]  # Used for plotting graph about Kalman Filter 2
        K3 = [self.sensor.Comp_Angle_X]  # Used for plotting graph about Kalman Filter 3

        while i < self.Samples:

            t0 = time.clock_gettime_ns(time.CLOCK_REALTIME)

            data = self.sensor.get_6axis(dt)

            t.append(t[-1] + dt)
            X.append(self.sensor.Gyr_Angle_X)
            Y.append(self.sensor.Gyr_Angle_Y)
            Z.append(self.sensor.Gyr_Angle_Z)
            Xcomp.append(self.sensor.Comp_Angle_X)
            Ycomp.append(self.sensor.Comp_Angle_Y)
            Zcomp.append(self.sensor.Comp_Angle_Z)
            K1.append(kalman_Filter_1.calculateangle(newangle = self.sensor.Acc_Angle_X, newrate = self.sensor.Gyr_Rate_X, dt=dt))
            K2.append(kalman_Filter_2.calculateangle(newangle = self.sensor.Acc_Angle_X, newrate = self.sensor.Gyr_Rate_X, dt=dt))
            K3.append(kalman_Filter_3.calculateangle(newangle = self.sensor.Acc_Angle_X, newrate = self.sensor.Gyr_Rate_X, dt=dt))

            i = i + 1

            # When the loop is finished, wait until the full time is Elapsed
            dt = 0
            while dt < self.Timer:
                t1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                dt = t1 - t0  # duration is integer with nanosecond resolution
            dt = dt / pow(10, 9)

        graph_title = ('MPU6050 Gyro drift over time \n Sampling Frequency = '
            + str(self.sampling_frequency) + 'Hz  +  MPU6050 Acc Range = ' + str(self.sensor.read_accel_range()) +
                      ' g - Gyr Range = ' + str(self.sensor.read_gyro_range()) + ' °/s - LP BW = ' + str(self.sensor.read_filter_range()) + ' Hz')

        my_legend = ('Compl. Filter' + str(self.sensor.Comp_Gyr_factor) + '/' + str(self.sensor.Comp_Acc_factor),
                     'Kal. Q_Angle ' + str(kalman_Filter_1.Q_angle) + ' Q_Bias ' + str(kalman_Filter_1.Q_angle) + ' R_meas ' + str(kalman_Filter_1.R_measure),
                     'Kal. Q_Angle ' + str(kalman_Filter_2.Q_angle) + ' Q_Bias ' + str(kalman_Filter_2.Q_angle) + ' R_meas ' + str(kalman_Filter_2.R_measure),
                     'Kal. Q_Angle ' + str(kalman_Filter_3.Q_angle) + ' Q_Bias ' + str(kalman_Filter_3.Q_angle) + ' R_meas ' + str(kalman_Filter_3.R_measure))
        plt.plot(t , Xcomp, color='black')
        plt.plot(t , K1, color='red')
        plt.plot(t , K2, color='green')
        plt.plot(t , K3, color='blue')
        plt.legend(my_legend)
        plt.xlabel('Time (s)')
        plt.ylabel('Angle displacement (°)')
        plt.title(graph_title)
        plt.grid()
        plt.show()
        plt.close()