#
# OpenMV Software for TDPS self-driving vehicle project
# The Line Tracking Module
# Dead Reckoning
# Sidi Liang, 2023
#



import time

class DeadReckoning:

    def __init__(self):
        self.time_last_dead_reckoning = 0
        self.v_x_last, self.v_y_last, self.v_z_last = 0, 0, 0
        self.a_x_last, self.a_y_last, self.a_z_last = 0, 0, 0
        self.p_x_last, self.p_y_last, self.p_z_last = 0, 0, 0

    def dead_reckoning(self, imu):
        # In local coordinates, i.e. x component of acceleration
        # will be in the sensor's x axis
        a_x, a_y, a_z = 0, 0, 0
        accel_window = 0.8
        sample_count = 10
        no_acc_count = 0
        no_movement_threshold = 5
        time_now_dead_reckoning = time.time_ns() * 1000000 #ms
        interval = time_now_dead_reckoning - self.time_last_dead_reckoning

        for i in range(sample_count):
            #Take the average to reduce error
            acc_x, acc_y, acc_z = imu.accelerometer() #m/s^2
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
        self.position_x, self.position_y, self.position_z = v_x, v_y, v_z

        self.a_x_last, self.a_y_last, self.a_z_last = a_x, a_y, a_z
        self.v_x_last, self.v_y_last, self.v_z_last = v_x, v_y, v_z
        self.p_x_last, self.p_y_last, self.p_z_last = p_x, p_y, p_z

        self.time_last_dead_reckoning = time_now_dead_reckoning

        return p_x, p_y, p_z
