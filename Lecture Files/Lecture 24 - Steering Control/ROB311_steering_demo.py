import sys
import threading
import time
import numpy as np
from threading import Thread
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype, mo_pid_params_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from rtplot import client
from scipy.signal import butter, lfilter, filtfilt
from simple_pid import PID
from pyPS4Controller.controller import Controller
import board
import adafruit_dotstar as dotstar
from enum import Enum
from collections import deque
from DataLogger import dataLogger

import FIR as fir
"""
ROB 311 - Ball-bot steering demo

This program uses a soft realtime loop to enforce loop timing. Soft real time loop is a  class
designed to allow clean exits from infinite loops with the potential for post-loop cleanup operations executing.

The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
A typical usage would set function_in_loop to be a method of an object, so that the object could store program state.
See the 'ifmain' for two examples.

Authors: Senthur Raj, Gray Thomas, Yves Nazon and Elliott Rouse 
Neurobionics Lab / Locomotor Control Lab
"""

import signal
import time
from math import sqrt

PRECISION_OF_SLEEP = 0.0001

# Version of the SoftRealtimeLoop library
__version__ = "1.0.0"

class LoopKiller:
    def __init__(self, fade_time=0.0):
        signal.signal(signal.SIGTERM, self.handle_signal)
        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGHUP, self.handle_signal)
        self._fade_time = fade_time
        self._soft_kill_time = None

    def handle_signal(self, signum, frame):
        self.kill_now = True

    def get_fade(self):
        # interpolates from 1 to zero with soft fade out
        if self._kill_soon:
            t = time.time() - self._soft_kill_time
            if t >= self._fade_time:
                return 0.0
            return 1.0 - (t / self._fade_time)
        return 1.0

    _kill_now = False
    _kill_soon = False

    @property
    def kill_now(self):
        if self._kill_now:
            return True
        if self._kill_soon:
            t = time.time() - self._soft_kill_time
            if t > self._fade_time:
                self._kill_now = True
        return self._kill_now

    @kill_now.setter
    def kill_now(self, val):
        if val:
            if self._kill_soon:  # if you kill twice, then it becomes immediate
                self._kill_now = True
            else:
                if self._fade_time > 0.0:
                    self._kill_soon = True
                    self._soft_kill_time = time.time()
                else:
                    self._kill_now = True
        else:
            self._kill_now = False
            self._kill_soon = False
            self._soft_kill_time = None

class SoftRealtimeLoop:
    def __init__(self, dt=0.001, report=False, fade=0.0):
        self.t0 = self.t1 = time.time()
        self.killer = LoopKiller(fade_time=fade)
        self.dt = dt
        self.ttarg = None
        self.sum_err = 0.0
        self.sum_var = 0.0
        self.sleep_t_agg = 0.0
        self.n = 0
        self.report = report

    def __del__(self):
        if self.report:
            print("In %d cycles at %.2f Hz:" % (self.n, 1.0 / self.dt))
            print("\tavg error: %.3f milliseconds" % (1e3 * self.sum_err / self.n))
            print(
                "\tstddev error: %.3f milliseconds"
                % (
                    1e3
                    * sqrt((self.sum_var - self.sum_err**2 / self.n) / (self.n - 1))
                )
            )
            print(
                "\tpercent of time sleeping: %.1f %%"
                % (self.sleep_t_agg / self.time() * 100.0)
            )

    @property
    def fade(self):
        return self.killer.get_fade()

    def run(self, function_in_loop, dt=None):
        if dt is None:
            dt = self.dt
        self.t0 = self.t1 = time.time() + dt
        while not self.killer.kill_now:
            ret = function_in_loop()
            if ret == 0:
                self.stop()
            while time.time() < self.t1 and not self.killer.kill_now:
                if signal.sigtimedwait(
                    [signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0
                ):
                    self.stop()
            self.t1 += dt
        print("Soft realtime loop has ended successfully.")

    def stop(self):
        self.killer.kill_now = True

    def time(self):
        return time.time() - self.t0

    def time_since(self):
        return time.time() - self.t1

    def __iter__(self):
        self.t0 = self.t1 = time.time() + self.dt
        return self

    def __next__(self):
        if self.killer.kill_now:
            raise StopIteration

        while (
            time.time() < self.t1 - 2 * PRECISION_OF_SLEEP and not self.killer.kill_now
        ):
            t_pre_sleep = time.time()
            time.sleep(
                max(PRECISION_OF_SLEEP, self.t1 - time.time() - PRECISION_OF_SLEEP)
            )
            self.sleep_t_agg += time.time() - t_pre_sleep

        while time.time() < self.t1 and not self.killer.kill_now:
            if signal.sigtimedwait([signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0):
                self.stop()
        if self.killer.kill_now:
            raise StopIteration
        self.t1 += self.dt
        if self.ttarg is None:
            # inits ttarg on first call
            self.ttarg = time.time() + self.dt
            # then skips the first loop
            return self.t1 - self.t0
        error = time.time() - self.ttarg  # seconds
        self.sum_err += error
        self.sum_var += error**2
        self.n += 1
        self.ttarg += self.dt
        return self.t1 - self.t0

# ---------------------------------------------------------------------------

class BBController(Controller):
    """
    This controller class can be implemented in many different ways and this is one of them.
    """
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.MAX_TZ = 0.5 # Nm
        self.MAX_VELOCITY = 0.85 # rad/sec

        self.DELTA_KP = 0.1
        self.DELTA_KD = 0.01

        self.MAX_ROTATION_TIME = 0.75 # Sec
        self.Tz = 0.0

        self.dphi_y_sp = 0.0
        self.dphi_x_sp = 0.0

        self.theta_kp = 9.0
        self.theta_ki = 0.0
        self.theta_kd = 0.1

        self.COOLDOWN = 0.5
        self.MAX_ROTATION_ITER = int(self.MAX_ROTATION_TIME/DT)

    def on_R2_press(self, value):
        self.dphi_y_sp = 1.0 * self.MAX_VELOCITY * (1.0 + value/JOYSTICK_SCALE)/2.0

    def on_R2_release(self):
        # TODO: Release mechanism
        self.dphi_y_sp = 0.0

    def on_L2_press(self, value):
        self.dphi_y_sp = -1.0 * self.MAX_VELOCITY * (1.0 + value/JOYSTICK_SCALE)/2.0

    def on_L2_release(self):
        # TODO: Release mechanism
        self.dphi_y_sp = 0.0

    def on_R1_press(self):
        for i in range(0, self.MAX_ROTATION_ITER):
            self.Tz = self.MAX_TZ * np.sin(i)
            time.sleep(DT)

        time.sleep(self.COOLDOWN)
    
    def on_R1_release(self):
        self.Tz = 0.0

    def on_L1_press(self):
        for i in range(0, self.MAX_ROTATION_ITER):
            self.Tz = -1.0 * self.MAX_TZ * np.sin(i)
            time.sleep(DT)

        time.sleep(self.COOLDOWN)
    
    def on_L1_release(self):
        self.Tz = 0.0

    def on_triangle_press(self):
        self.theta_kp += self.DELTA_KP

        if self.theta_kp > MAX_THETA_KP:
            self.theta_kp = MAX_THETA_KP
            print("Maxed out Theta Kp at {:.2f}.".format(self.theta_kp))
        else:
            print("Increased Theta Kp to {:.2f}.".format(self.theta_kp))

    def on_triangle_release(self):
        pass

    def on_x_press(self):
        self.theta_kp -= self.DELTA_KP

        if self.theta_kp < MIN_THETA_KP:
            self.theta_kp = MIN_THETA_KP
            print("Bottomed out Theta Kp at {:.2f}.".format(self.theta_kp))
        else:
            print("Decreased Theta Kp to {:.2f}".format(self.theta_kp))

    def on_x_release(self):
        pass

    def on_circle_press(self):
        self.theta_kd += self.DELTA_KD
        if self.theta_kd > MAX_THETA_KD:
            self.theta_kd = MAX_THETA_KD
            print("Maxed out Theta Kd at {:.2f}.".format(self.theta_kd))
        else:        
            print("Increased Theta Kd to {:.2f}.".format(self.theta_kd))

    def on_circle_release(self):
        pass

    def on_square_press(self):
        self.theta_kd -= self.DELTA_KD
        if self.theta_kd < MIN_THETA_KD:
            self.theta_kd = MIN_THETA_KD
            print("Bottomed out Theta Kd at {:.2f}.".format(self.theta_kd))
        else:        
            print("Decreased Theta Kd to {:.2f}".format(self.theta_kd))

    def on_square_release(self):
        pass

    def on_options_press(self):
        print("Exiting controller thread.")
        sys.exit()

# ---------------------------------------------------------------------------

JOYSTICK_SCALE = 32767

FREQ = 200
DT = 1/FREQ

RW = 0.048
RK = 0.1210
ALPHA = np.deg2rad(45)
MK = 0.62369
IK = 0.00365

Fs = FREQ # Sampling rate in Hz
Fc = 0.8 # Cut-off frequency of the filter in Hz

Fn = Fc/Fs # Normalized equivalent of Fc
N = 100 # Taps of the filter

# ---------------------------------------------------------------------------

lowpass_filter_dphi_x = fir.FIR()
lowpass_filter_dphi_x.lowpass(N, Fn)

lowpass_filter_dphi_y = fir.FIR()
lowpass_filter_dphi_y.lowpass(N, Fn)

MAX_THETA = np.deg2rad(4) # Maximum lean angle: 4 degrees

# MAX PLANAR DUTY SHOULD BE LESS THAN 1.0
MAX_STA_DUTY = 0.6
MAX_VEL_DUTY = 0.4

MAX_DPHI = 4.0 # rad/sec
MAX_DDPHI = 40.0 # rad/sec^2

DPHI_DEADBAND = 0.5 # rad/sec

ROLL_THETA_KP = 9.0
ROLL_THETA_KI = 0.0
ROLL_THETA_KD = 0.05

PITCH_THETA_KP = 9.0
PITCH_THETA_KI = 0.0
PITCH_THETA_KD = 0.05

PHI_KP = 0.5
PHI_KI = 0.0
PHI_KD = 0.0

MAX_THETA_KP = 16.0
MIN_THETA_KP = 7.0

MAX_THETA_KD = 2.0
MIN_THETA_KD = -2.0

WMA_WEIGHTS = np.array([20, 40, 60, 80])
WMA_WINDOW_SIZE = len(WMA_WEIGHTS)
WMA_NORM = WMA_WEIGHTS/np.sum(WMA_WEIGHTS)

def wma_filter(wma_window):
    return np.sum(WMA_NORM * wma_window)

def register_topics(ser_dev:SerialProtocol):
    # Mo :: Commands, States
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

def transform_w2b(m1, m2, m3):
    """
    Returns Phi attributes
    """

    x = 0.323899 * m2 - 0.323899 * m3
    y = -0.374007 * m1 + 0.187003 * m2 + 0.187003 * m3
    z = 0.187003 * m1 + 0.187003 * m2 + 0.187003 * m3

    return x, y, z

if __name__ == "__main__":

    trial_num = int(input('Trial Number? '))
    filename = 'ROB311_Velocity_Test_%i' % trial_num
    dl = dataLogger(filename + '.txt')

    t_start = 0.0

    ser_dev = SerialProtocol()
    register_topics(ser_dev)

    # Init serial
    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # Local structs
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    commands['kill'] = 0.0
    zeroed = False

    psi = np.zeros((3, 1))
    psi_offset = np.zeros((3, 1))

    phi = np.zeros((3, 1))
    prev_phi = phi

    theta_x = 0.0 # filtered
    theta_y = 0.0 # filtered

    dphi_x = 0.0 # filtered
    dphi_y = 0.0 # filtered

    dpsi = np.zeros((3, 1))

    dphi = np.zeros((3, 1))
    prev_dphi = np.zeros((3, 1))

    ddphi = np.zeros((3, 1))

    # deque is a data structure that automatically pops the previous data based on its max length.
    theta_x_window = deque(maxlen=WMA_WINDOW_SIZE) # A sliding window of values
    theta_y_window = deque(maxlen=WMA_WINDOW_SIZE) # A sliding window of values

    for _ in range(WMA_WINDOW_SIZE):
        theta_x_window.append(0.0)
        theta_y_window.append(0.0)

    theta_roll_pid_components = np.array([0.0, 0.0, 0.0])
    theta_pitch_pid_components = np.array([0.0, 0.0, 0.0])

    phi_roll_pid_components = np.array([0.0, 0.0, 0.0])
    phi_pitch_pid_components = np.array([0.0, 0.0, 0.0])

    # Net Tx, Ty, and Tz 
    Tx = 0.0
    Ty = 0.0
    Tz = 0.0

    # Steering controller torques: Tx_e, Ty_e, and Tz_e
    Tx_e = 0.0
    Ty_e = 0.0
    Tz_e = 0.0

    # T1, T2, and T3
    T1 = 0.0
    T2 = 0.0
    T3 = 0.0

    # Time for comms to sync
    time.sleep(1.0)

    # Send the gains 
    ser_dev.send_topic_data(101, commands)

    theta_roll_sp = 0.0
    theta_pitch_sp = 0.0

    phi_roll_sp = 0.0
    phi_pitch_sp = 0.0
    
    # Initializing PID classes for the stability controller (along x|roll and y|pitch).
    theta_roll_pid = PID(ROLL_THETA_KP, ROLL_THETA_KI, ROLL_THETA_KD, theta_roll_sp)
    theta_pitch_pid = PID(PITCH_THETA_KP, PITCH_THETA_KI, PITCH_THETA_KD, theta_pitch_sp)

    theta_roll_pid.output_limits = (-MAX_STA_DUTY, MAX_STA_DUTY)
    theta_pitch_pid.output_limits = (-MAX_STA_DUTY, MAX_STA_DUTY)

    # Initializing PID classes for the steering controller (along x|roll and y|pitch).
    phi_roll_pid = PID(PHI_KP, PHI_KI, PHI_KD, phi_roll_sp)
    phi_pitch_pid = PID(PHI_KP, PHI_KI, PHI_KD, phi_pitch_sp)

    phi_roll_pid.output_limits = (-MAX_VEL_DUTY, MAX_VEL_DUTY)
    phi_pitch_pid.output_limits = (-MAX_VEL_DUTY, MAX_VEL_DUTY)
    
    print('Starting the controller!')
    i = 0

    # This thread runs in parallel to the main controller loop and listens for any PS4 input
    bb_controller = BBController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    bb_controller_thread = threading.Thread(target=bb_controller.listen, args=(10,))
    bb_controller_thread.start()

    for t in SoftRealtimeLoop(dt=DT, report=True):

        try:
            states = ser_dev.get_cur_topic_data(121)[0]

        except KeyError as e:
            # Calibration: 10 seconds
            print("<< CALIBRATING :: {:.2f} >>".format(t))
            continue

        # Data from the Pico
        psi[0] = states['psi_1']
        psi[1] = states['psi_2']
        psi[2] = states['psi_3']

        dpsi[0] = states['dpsi_1']
        dpsi[1] = states['dpsi_2']
        dpsi[2] = states['dpsi_3']

        # A sliding window of "WMA_WINDOW_SIZE" values for the WMA filter
        theta_x_window.append(states['theta_roll'])
        theta_y_window.append(states['theta_pitch'])

        # Applying a WMA filter on the IMU values
        theta_x = wma_filter(theta_x_window)
        theta_y = wma_filter(theta_y_window)

        # A ten second wait to place the bot on top of the ball--this is to 
        # reset the encoder values so that at i=0, the ball-bot's position is (0, 0)
        if t > 11.0 and t < 21.0:
            print("<< PLACE THE BOT ON TOP OF THE BALL :: {:.2f} >>".format(t))
        elif t > 21.0:
            if not zeroed:
                psi_offset = psi
                zeroed = True

        psi = psi - psi_offset

        # Transforming wheel attributes (position and velocity) to ball attributes.
        phi[0], phi[1], phi[2] = transform_w2b(psi[0], psi[1], psi[2])
        dphi[0], dphi[1], dphi[2] = transform_w2b(dpsi[0], dpsi[1], dpsi[2])

        # Lowpass filtering the ball-velocity estimates
        dphi_x = lowpass_filter_dphi_x.filter(dphi[0][0])
        dphi_y = lowpass_filter_dphi_y.filter(dphi[1][0])

        if zeroed:
            if i == 0:
                t_start = time.time()

            i = i + 1
            t_now = time.time() - t_start

        # Start the steering controller if there is a change in the 
        # ball-velocity setpoint using the PS4 controller.
        if np.abs(bb_controller.dphi_y_sp) > DPHI_DEADBAND:
            phi_pitch_pid.setpoint = bb_controller.dphi_y_sp
            phi_roll_pid.setpoint = 0.0

            Tx_e = phi_roll_pid(dphi_x)
            Ty_e = phi_pitch_pid(dphi_y)

        # Also start the steering controller if the ball-velocity is greater than
        # DPHI_DEADBAND (0.5 rad/sec) to prevent the ball-bot from drifting
        elif np.abs(dphi_x) > DPHI_DEADBAND or np.abs(dphi_y) > DPHI_DEADBAND:
            phi_roll_pid.setpoint = 0.0
            phi_pitch_pid.setpoint = 0.0

            Tx_e = phi_roll_pid(dphi_x)
            Ty_e = phi_pitch_pid(dphi_y)

        else:
            Tx_e = 0.0
            Ty_e = 0.0

        # Max Lean angle (Theta) constraint: If theta is greater than the maximum lean angle 
        # (4 degrees) for our ball-bot, then turn off the steering controller.
        if np.abs(theta_x) > MAX_THETA or np.abs(theta_y) > MAX_THETA:
            Tx_e = 0.0
            Ty_e = 0.0

        # Summation of planar torques
        # Stability controller + Steering controller
        Tx = theta_roll_pid(theta_x) + Tx_e
        Ty = theta_pitch_pid(theta_y) + Ty_e
        Tz = bb_controller.Tz

        # Conversion of planar torques to motor torques
        T1 = (-0.3333) * (Tz - (2.8284 * Ty))
        T2 = (-0.3333) * (Tz + (1.4142 * (Ty + 1.7320 * Tx)))
        T3 = (-0.3333) * (Tz + (1.4142 * (Ty - 1.7320 * Tx)))

        # Sending motor torque commands to the pico
        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3

        # 
        theta_roll_pid_components[0], theta_roll_pid_components[1], theta_roll_pid_components[2] = theta_roll_pid.components
        theta_pitch_pid_components[0], theta_pitch_pid_components[1], theta_pitch_pid_components[2] = theta_pitch_pid.components

        phi_roll_pid_components[0], phi_roll_pid_components[1], phi_roll_pid_components[2] = phi_roll_pid.components
        phi_pitch_pid_components[0], phi_pitch_pid_components[1], phi_pitch_pid_components[2] = phi_pitch_pid.components

        ser_dev.send_topic_data(101, commands)

        if zeroed:
            print(" << Iteration no: {}, DPHI X: {:.2f}, DPHI Y: {:.2f} >>".format(i, dphi[0][0], dphi[1][0]))
            # Construct the data matrix for saving - you can add more variables by replicating the format below
            data = [i] + [t_now] + \
                [states['theta_roll']] + [states['theta_pitch']] + \
                    [Tx] + [Ty] + [Tz] + \
                        [T1] + [T2] + [T3] + \
                            [phi[0][0]] + [phi[1][0]] + [phi[2][0]] + \
                                [psi[0][0]] + [psi[1][0]] + [psi[2][0]] + \
                                    [theta_roll_pid_components[0]] + [theta_roll_pid_components[1]] + [theta_roll_pid_components[2]] + \
                                        [theta_pitch_pid_components[0]] + [theta_pitch_pid_components[1]] + [theta_pitch_pid_components[2]] + \
                                            [phi_roll_pid_components[0]] + [phi_roll_pid_components[1]] + [phi_roll_pid_components[2]] + \
                                                [phi_pitch_pid_components[0]] + [phi_pitch_pid_components[1]] + [phi_pitch_pid_components[2]] + \
                                                    [dphi[0][0]] + [dphi[1][0]] + [dphi[2][0]] + \
                                                        [dphi_x] + [dphi_y] + [theta_x] + [theta_y] + \
                                                            [Tx_e] + [Ty_e]

            dl.appendData(data)

    print("Resetting Motor commands.")
    time.sleep(0.25)
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    time.sleep(0.25)
    commands['kill'] = 1.0
    time.sleep(0.25)
    ser_dev.send_topic_data(101, commands)
    time.sleep(0.25)

    dl.writeOut()
