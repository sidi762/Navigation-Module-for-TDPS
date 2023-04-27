#
# OpenMV Software for TDPS self-driving vehicle project
# The Line Tracking Module
# Dead Reckoning
# Sidi Liang, 2023
#



import pyb

class DeadReckoning:

    def __init__(self):
        self._time_init = pyb.millis() # miliseconds
        print("Dead Reckoning initialized at ", self._time_init)
        self.time_last_dead_reckoning = 0
        self.v_x_last, self.v_y_last, self.v_z_last = 0, 0, 0
        self.a_x_last, self.a_y_last, self.a_z_last = 0, 0, 0
        self.p_x_last, self.p_y_last, self.p_z_last = 0, 0, 0
        self.velocity_x, self.velocity_y, self.velocity_z = 0, 0, 0
        self.position_x, self.position_y, self.position_z = 0, 0, 0

    def dead_reckoning(self, imu):
        # In local coordinates, i.e. x component of acceleration
        # will be in the sensor's x axis
        a_x, a_y, a_z = 0, 0, 0
        accel_window = 0.05
        sample_count = 10
        no_acc_count = 0
        no_movement_threshold = 4
        time_now_dead_reckoning = pyb.millis() #ms
        time_after_init = time_now_dead_reckoning - self._time_init
        if  time_after_init < 3000:
            #Wait for calibration
            #print("Dead Reckoning waiting for calibration")
            #print(3000 - time_after_init, " ms remaining")
            return 0, 0, 0
        interval = (time_now_dead_reckoning - self.time_last_dead_reckoning) / 1000 #seconds

        for i in range(sample_count):
            #Take the average to reduce error
            acc_x, acc_y, acc_z = imu.linear_acceleration() #m/s^2
            #acc_x, acc_y, acc_z = imu.accelerometer
            a_x += acc_x
            a_y += acc_y
            a_z += acc_z

        a_x /= sample_count
        a_y /= sample_count
        a_z /= sample_count

        if -accel_window < a_x < accel_window:
            a_x = 0
        if -accel_window < a_y < accel_window:
            a_y = 0
        if -accel_window < a_z < accel_window:
            a_z = 0

        if a_x == 0 and a_y == 0 and a_z == 0:
            no_acc_count += 1
        else:
            no_acc_count = 0

        if no_acc_count > no_movement_threshold:
            self.v_x_last = 0
            self.v_y_last = 0
            self.v_z_last = 0
            no_acc_count = 0

        #print(a_x)
        v_dx = a_x  * interval
        v_dy = a_y  * interval
        v_dz = a_z  * interval
        v_x = v_dx + self.v_x_last
        v_y = v_dy + self.v_y_last
        v_z = v_dz + self.v_z_last
        self.velocity_x, self.velocity_y, self.velocity_z = v_x, v_y, v_z

        p_x = v_dx * interval + self.p_x_last
        p_y = v_dy * interval + self.p_y_last
        p_z = v_dz * interval + self.p_z_last
        self.position_x, self.position_y, self.position_z = p_x, p_y, p_z

        self.a_x_last, self.a_y_last, self.a_z_last = a_x, a_y, a_z
        self.v_x_last, self.v_y_last, self.v_z_last = v_x, v_y, v_z
        self.p_x_last, self.p_y_last, self.p_z_last = p_x, p_y, p_z

        self.time_last_dead_reckoning = time_now_dead_reckoning

        return p_x, p_y, p_z
