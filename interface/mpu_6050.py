from mpu6050 import mpu6050
from math import *
# from simple_pid import PID
import time

class PID:
    """
    A PID controller sporting an option to do accumulator min/max anti-windup.
    """

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, reference=None, iMin=None, iMax=None, anti_windup=False, initial=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.reference = reference
        self.previous_error = 0.0 if (reference is None or initial is None) else (reference - initial)
        self.accumulated_error = 0.0
        self.anti_windup = anti_windup
        self.accumulator_min = iMin
        self.accumulator_max = iMax

    def control(self, input, dt=1, reference=None):
        """
        Compute a control value for @input. If no @reference is used, fall
        back to self.reference. If no @dt is provided, fall back to discrete 1.
        """
        # Calculate new error and accumulate
        error = (self.reference if reference is None else reference) - input
        self.accumulated_error += error * dt
        error_diff = (error - self.previous_error) / dt
        # Check for accumulator limits
        if (self.anti_windup):
            if self.accumulated_error < self.accumulator_min:
                self.accumulated_error = self.accumulator_min
            elif self.accumulated_error > self.accumulator_max:
                self.accumulated_error = self.accumulator_max
        # Calculate control output
        P_term = self.Kp * error
        D_term = self.Kd * error_diff
        I_term = self.Ki * self.accumulated_error
        control = P_term + I_term + D_term
        # Store current error
        self.previous_error = error
        # Return control value
        return control

    def anti_windup(self, acc_min, acc_max=None):
        """
        @acc_min false for disabling
        @acc_max defaults to -@acc_min
        """
        self.anti_windup = True if acc_min is not False else False
        self.accumulator_min = acc_min
        self.accumulator_max = acc_max if acc_max is not None else -acc_min


imu = mpu6050(0x68)

update_rate = 1

desired_pitch = 0.0
desired_roll = 0.0

pitch_gains = {'kp': 1.2, 'ki': 0.02, 'kd': 0.2, 'iMin': -0.02, 'iMax': 1, 'anti_windup': True} # Pitch PID

roll_gains = {'kp': 1.2, 'ki': 0.02, 'kd': 0.2,'iMin': -2, 'iMax': 2, 'anti_windup': True}#Roll PID

pitch_PID = PID(Kp=pitch_gains['kp'], Ki=pitch_gains['ki'], Kd=pitch_gains['kd'], iMin=pitch_gains['iMin'], iMax=pitch_gains['iMax'], anti_windup= pitch_gains['anti_windup'])
roll_PID = PID(Kp=roll_gains['kp'], Ki=roll_gains['ki'], Kd=roll_gains['kd'], iMin=roll_gains['iMin'], iMax=roll_gains['iMax'], anti_windup= roll_gains['anti_windup'])


def get_roll_pitch_angle():
    accel = imu.get_accel_data()
    accel_X, accel_Y, accel_Z = accel['x'], accel['y'], accel['z']
    pitch = 180 * atan2(accel_X, sqrt(accel_Y*accel_Y + accel_Z*accel_Z))/ pi
    roll = 180 * atan2(accel_Y, sqrt(accel_X*accel_X + accel_Z*accel_Z))/ pi
    return roll, pitch


while True:
    current_roll, current_pitch = get_roll_pitch_angle()

    pitch_pid_output = pitch_PID.control(current_pitch, reference=desired_pitch)
    roll_pid_output = roll_PID.control(current_roll, reference=desired_roll)

    print(current_pitch, pitch_pid_output)

    # time.sleep(0.01)





