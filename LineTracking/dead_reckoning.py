#
# OpenMV Software for TDPS self-driving vehicle project
# The Line Tracking Module
# Dead Reckoning
# Sidi Liang, 2023
#



import time

time_last_dead_reckoning = 0
v_x_last, v_y_last, v_z_last = 0, 0, 0
a_x_last, a_y_last, a_z_last = 0, 0, 0
p_x_last, p_y_last, p_z_last = 0, 0, 0

def dead_reckoning(imu):
    # In local coordinates, i.e. x component of acceleration
    # will be in the sensor's x axis
    a_x, a_y, a_z = 0, 0, 0
    accel_window = 50
    sample_count = 10
    no_acc_count = 0
    no_movement_threshold = 5
    time_now_dead_reckoning = time.time_ns() * 1000000 #ms
    interval = time_now_dead_reckoning - time_last_dead_reckoning

    for i in range(sample_count):
        #Take the average to reduce error
        acc_x, acc_y, acc_z = imu.accelerometer()
        a_x += acc_x
        a_y += acc_y
        a_z += acc_z

    a_x /= sample_count
    a_y /= sample_count
    a_z /= sample_count

    if -accel_window < a_x < accel_window:
        a_x = 0
    else:
        a_x *= 0.00981
    if -accel_window < a_y < accel_window:
        a_y = 0
    else:
        a_y *= 0.00981
    if -accel_window < a_z < accel_window:
        a_z = 0
    else:
        a_z *= 0.00981

    if a_x == 0 and a_y == 0 and a_z == 0:
        no_acc_count += 1
    else:
        no_acc_count = 0

    if no_acc_count > no_movement_threshold:
        v_x_last = 0
        v_y_last = 0
        v_z_last = 0
        no_acc_count = 0

    v_x = v_x_last + (a_x_last + (a_x - a_x_last) / 2.0) * interval
    v_y = v_y_last + (a_y_last + (a_y - a_y_last) / 2.0) * interval
    v_z = v_z_last + (a_z_last + (a_z - a_z_last) / 2.0) * interval

    p_x = p_x_last + (v_x_last + (v_x - v_x_last) / 2.0) * interval
    p_y = p_y_last + (v_y_last + (v_y - v_y_last) / 2.0) * interval
    p_z = p_z_last + (v_z_last + (v_z - v_z_last) / 2.0) * interval

    a_x_last, a_y_last, a_z_last = a_x, a_y, a_z
    v_x_last, v_y_last, v_z_last = v_x, v_y, v_z
    p_x_last, p_y_last, p_z_last = p_x, p_y, p_z

    time_last_dead_reckoning = time_now_dead_reckoning
    
    return p_x, p_y, p_z
