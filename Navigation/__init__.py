#
# OpenMV Software for TDPS self-driving vehicle project
# The Navigation Module
# Sidi Liang, 2023
#

#Notes (to be removed):
#kernel_size = 1 # kernel width = (size*2)+1, kernel height = (size*2)+1
#kernel = [-9, +9, +11,\
#          -23, +40, +13,\
#          -21, -19, -1]

#kernel = [-3, +0, +1,\
#          -4, +8, +2,\
#          -3, -2, +1]
#thresholds = [(100, 255)]

import image, time, uasyncio
from dead_reckoning import DeadReckoning
from odometer import Odometer
from bno055 import BNO055, AXIS_P7
from pid import PID

class Navigator:

    def __init__(self, imu, status_data_ref,
                 turn_pid_p = 0.01, turn_pid_i = 0.005,
                 turn_pid_d = 0, turn_pid_imax = 0):
        self._imu = imu
        self._status_data_ref = status_data_ref
        self._turn_pid = PID(p = turn_pid_p,
                             i = turn_pid_i,
                             d = turn_pid_d,
                             imax = turn_pid_imax)


    _target_heading = 0
    _current_heading = 0
    _is_navigating = False
    _control_output = 0

    def _update_status_data(self, control):
        self._status_data_ref['Control_PID'] = control

    def _update_current_heading_from_imu(self, imu):
        yaw, roll, pitch = imu.euler()
        #Note: Need to revisit this for the potential issues with calibration
        self._current_heading = yaw
        return yaw

    def _clip_turn_error(self, turn_err):
        if turn_err < 0:
            turn_err += 360
        return turn_err

    def _calculate_direction(self, current_heading, target_heading):
        turn_err = 0
        turn_err_right = target_heading  - current_heading
        self._clip_turn_error(turn_err_right)
        turn_err_left = current_heading - target_heading
        self._clip_turn_error(turn_err_left)
        if turn_err_right <= turn_err_left:
            # Turn right
            return 1
        else:
            # Turn left
            return -1

    def navigate(self, target_heading = self._target_heading):
        '''
            Navigate using PID
            Returns the control output
        '''
        if self._imu:
            current_heading = _update_current_heading_from_imu(self._imu)
            direction = self._calculate_direction(current_heading, target_heading)
            turn_err = direction * (target_heading  - current_heading)
            turn_control = self._turn_pid.get_pid(turn_err, 1)
            self._control_output = turn_control
            return turn_control
        else:
            return 0

    def turn_to_heading(self, target_heading, direction = 0):
        '''
            Turn to a given heading
            Turns right for direction = 1, left for direction = -1,
            auto for direction = 0
            Returns when the turn is completed
        '''
        current_heading = self._update_current_heading_from_imu()
        if direction == 0:
            direction = self._calculate_direction(current_heading, target_heading)

        degress = math.abs(target_heading - current_heading)
        self.turn_degress(degress, direction)

        return 0

    def turn_degrees(self, degrees, direction = 1):
        '''
            Turn for a given angle
            degress should be positive
            Turns right for direction = 1, left for direction = -1
            Returns when the turn is completed
        '''
        current_heading = self._update_current_heading_from_imu()
        target_heading = current_heading + degrees
        self._target_heading = target_heading
        print("Turning heading ", target_heading, "...")
        while target_heading != current_heading:
            turn_err = direction * (target_heading  - current_heading)
            if turn_err < 0:
                turn_err += 360
            turn_control = self._turn_pid.get_pid(turn_err, 1)
            self._control_output = turn_control
            self._update_status_data(turn_control)
            current_heading = self._update_current_heading_from_imu()

        print("Turn completed, current heading ", current_heading)
        return 0

    def turn_right_degrees(self, degrees):
        '''
            Turn right for a given angle
            Returns when the turn is completed
        '''
        self.turn_degrees(degrees, 1)
        return 0

    def turn_left_degrees(self, degrees):
        '''
            Turn left for a given angle
            Returns when the turn is completed
        '''
        self.turn_degrees(degrees, -1)
        return 0

    def turn_right_90(self):
        '''
            Turn right for 90 degrees
        '''
        self.turn_degrees(90, 1)
        return 0

    def turn_left_90(self):
        '''
            Turn left for 90 degress
        '''
        self.turn_degrees(90, -1)
        return 0

    async def _navigate_async(self):
        '''
            Navigate using PID
            This is a coroutine
        '''
        while self._is_navigating:
            self._control_output = self.navigate()
            await uasyncio.sleep(0)

        return 0

    def start_async(self):
        '''
            Check if imu is avilable and start the navigation
        '''
        if self._imu:
            self._is_navigating = True
            self._task = uasyncio.create_task(self._navigate_async())
            return 1
        else:
            self._is_navigating = False
            return 0

    def end_async(self):
        '''
            End the navigation and do some cleanups if necessary
        '''
        self._is_navigating = False

    def set_target_heading(self, heading):
        self._target_heading = heading

    def get_target_heading(self):
        return self._target_heading

    def get_current_heading(self):
        return self._current_heading

    def get_control_output(self):
        return self._control_output

class LineTracking:

    def __init__(self, sensor, kernal_size = 1,
                 kernal = [-3, +0, +1, -4, +8, +2, -3, -2, +1],
                 thresholds = [(100, 255)],
                 rho_pid_p = 0.4, rho_pid_i = 0, rho_pid_d = 0,
                 rho_pid_imax = 0,
                 theta_pid_p = 0.001, theta_pid_i = 0, theta_pid_d = 0,
                 theta_pid_imax = 0, line_mag_thrs = 3,
                 draw = False):
        """
            Class for line tracking

            Arguments:
                sensor: openMV sensor
                kernal_size: The kernal size for filtering, calculated by:
                             kernel width = (size*2)+1,
                             kernel height = (size*2)+1,
                             defaults to 1 (3x3 kernal).
                kernal: The kernal for filtering,
                        defaults to [-3, +0, +1,\
                                     -4, +8, +2,\
                                     -3, -2, +1]
                thresholds: The thresholds for binary filtering,
                            defaults to [(100, 255)].
                rho_pid_p: The P factor for the rho PID controller
                        (for reducing the error of rho of the line).
                rho_pid_i: The I factor for the rho PID controller.
                rho_pid_d: The D factor for the rho PID controller.
                rho_pid_imax: The imax for the rho PID controller.
                theta_pid_p: The P factor for the theta PID controller
                        (for reducing the error of theta of the line).
                theta_pid_i: The I factor for the theta PID controller.
                theta_pid_d: The D factor for the theta PID controller.
                theta_pid_imax: The imax factor for the theta PID
                                controller.
                line_mag_thrs: The threshold to determine if the line
                               is valid.
                draw: Set to True to draw the resulting line on the
                      captured image, defaults to False.

            Usage:
                line_tracking = LineTracking(sensor)
                line_tracking.start()
                while(True):
                    control = line_tracking.calculate()
                    line = line_tracking.get_line()
                line_tracking.stop()
        """
        self._sensor = sensor
        self._kernal_size = kernal_size
        self._kernal = kernal
        self._thresholds = thresholds
        self._sensor_settings = {'pixformat': 0, 'framesize': 0}
        self._is_started = False
        self._line_mag_thrs = line_mag_thrs
        self._draw = draw
        self._rho_pid = PID(p = rho_pid_p,
                            i = rho_pid_i,
                            d = rho_pid_d,
                            imax = rho_pid_imax)
        self._theta_pid = PID(p = theta_pid_p,
                              i = theta_pid_i,
                              d = theta_pid_d,
                              imax = theta_pid_imax)
        self._theta_err = 0
        self._rho_err = 0


    def start(self):
        """
            Stores the current state of the sensor and sets the sensor
            for line tracking
        """
        self._store_sensor_settings()
        self._set_sensor_for_line_detection()
        img = self._sensor.snapshot()
        self._center_coord = img.width() / 2
        self._is_started = True

    def end(self):
        """
            Resumes sensor to the stored settings
        """
        # Resume sensor settings
        sens = self._sensor
        sens.set_pixformat(self._sensor_settings.pixformat)
        sens.set_framesize(self._sensor_settings.framesize)
        sens.skip_frames(time = 100)     # Wait for settings take effect.
        self._is_started = False

    def _set_sensor_for_line_detection(self):
        sens = self._sensor
        sens.set_pixformat(self._sensor.GRAYSCALE) # Set pixel format to GRAYSCALE
        sens.set_framesize(self._sensor.HQVGA)
        #sens.set_windowing((60, 40, 160, 120))
        # if (self._sensor.get_id() == self._sensor.OV7725):
             # Set the sharpness/edge register for OV7725
        #    print("Using OV7725")
        #    sens.__write_reg(0xAC, 0xDF)
        #    sens.__write_reg(0x8F, 0xFF)
        sens.skip_frames(time = 100)     # Wait for settings take effect.

    def _store_sensor_settings(self):
        self._sensor_settings['pixformat'] = self._sensor.get_pixformat()
        self._sensor_settings['framesize'] = self._sensor.get_framesize()


    def _apply_filter(self, img, threshold=True):
        #img.gamma_corr(gamma=1.0)
        img.morph(self._kernal_size, \
                  self._kernal, \
                  threshold=threshold, \
                  offset=2, \
                  invert=True)
        #img.binary(self._thresholds)
        img.erode(1, threshold = 3)
        return img

    def _capture_filter_and_calculate_line(self):
        if not self._is_started:
            print("Error: Set the camera first by calling LineTracking.start()")
            return 0
        img = self._sensor.snapshot()
        img = self._apply_filter(img)
        line = img.get_regression([(255, 255)], robust=True)
        self._img = img
        if line is not None:
            if self._draw:
                img.draw_line(line.line(), color=(255,255,0))
            self._calculated_line = line
            self._line_magnitude = line.magnitude()
            return line
        else:
            # Should we return the last valid line here?
            # Should we update the _calculated_line here?
            self._calculated_line = None
            self._line_magnitude = 0
            return None

    def _line_error(self, line):
        if line is not None:
            # rho_err is greater than 0: vehicle is on the left side
            #                            of the line, should turn right
            # rho_err is less than 0: vehicle is on the right side
            #                         of the line, should turn left
            rho_err = abs(line.rho())-self._center_coord

            # 90 > theta_err > 0: should turn right to be parallel to
            #                     the line
            # 0 > theta_err > -90: should turn left to be parallel to
            #                     the line
            if line.theta()>90:
                theta_err = line.theta()-180
            else:
                theta_err = line.theta()
            self._rho_err, self._theta_err = rho_err, theta_err
            return rho_err, theta_err
        else:
            self._rho_err, self._theta_err = 0, 0
            return 0, 0

    def _cilp_velocity_command(self, v):
        if v == None:
            return 0
        else:
            if v > 100: return 100
            elif v < 0: return 0
            else: return v

    def get_theta_err(self):
        """
            Returns the theta error as in Hough transfrom
            range: (0, 180) degrees
        """
        theta_err = self._theta_err
        if theta_err < 0:
            theta_err += 180
        return theta_err

    def calculate(self):
        """
            Returns the control factor and the velocity command,
            produced by pid for the vehicle to follow the line
        """
        line = self._capture_filter_and_calculate_line()
        magnitude = self._line_magnitude
        magnitude_threshold = self._line_mag_thrs
        # Todo: finetune
        velocity_command = (magnitude - magnitude_threshold) * 3

        velocity_command = self._cilp_velocity_command(velocity_command)
        if magnitude < magnitude_threshold:
            # Todo: action to reaquire the line
            return 0, 10 #Slow down the vehicle when no valid line is available

        rho_err, theta_err = self._line_error(line)
        rho_control = self._rho_pid.get_pid(rho_err, 1)
        theta_control = self._theta_pid.get_pid(theta_err, 1)
        self._rho_control, self._theta_control = rho_control, theta_control
        control = rho_control + theta_control
        self._control = control
        self._velocity_command = velocity_command

        return control, velocity_command

    def get_line(self):
        """
            Returns the detected line as a Line object.
            Returns None if no line have ever been detected.
        """
        if self._calculated_line:
            return self._calculated_line
        else:
            return None

    def get_line_magnitude(self):
        """
            Returns the magnitude of the detected line.
        """
        if self._line_magnitude:
            return self._line_magnitude
        else:
            return 0
