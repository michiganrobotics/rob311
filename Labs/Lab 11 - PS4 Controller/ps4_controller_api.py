import sys
import threading
import numpy as np
from pyPS4Controller.controller import Controller

# ---------------------------------------------------------------------------
# ROB 311 PS4 Controller class
# Add these code snippets to your balance controller
# To use the torque commands from the PS4 controller, set the torques to values obtained
# from button presses. This script will print when some buttons are pressed and serves as an example
# using the PS4 controller.  This script also has three example methods for commanding torque using 
# the PS4 controller (rob311_bt_controller.tz_demo_X), which can be used to set the Tz torque in your
# balance script.
# 
# The soft real-time loop is already included in your balance control script.
# Authors: Senthur Raj, Gray Thomas, and Elliott Rouse

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

JOYSTICK_SCALE = 32767

FREQ = 200
DT = 1/FREQ

class ROB311BTController(Controller):
    def __init__(self, interface, connecting_using_ds4drv=False, event_definition=None, event_format=None):
        super().__init__(interface, connecting_using_ds4drv, event_definition, event_format)

        # ------------------------------------
        # Declare required attributes/values 

        # DEMO 1: Modifying Tz value with Triggers
        self.tz_demo_1 = 0.0

        # DEMO 2: Modifying Tz value with Right Thumbstick (UP/DOWN)
        self.tz_demo_2 = 0.0

        # DEMO 3: Modifying Tz value with Shoulder/Bumper Buttons
        self.tz_demo_3 = 0

        # ------------------------------------

    # Continuous value with Triggers

    def on_R2_press(self, value):
        # Normalizing raw values from [-1.0, 1.0] to [0.0, 1.0]
        self.tz_demo_1 = (1.0 + np.abs(value/JOYSTICK_SCALE))/2.0

    def on_R2_release(self):
        # Reset values
        self.tz_demo_1 = 0.0

    def on_L2_press(self, value):
        # Normalizing raw values from [-1.0, 1.0] to [-1.0, 0.0]
        self.tz_demo_1 = -1 * (1.0 + np.abs(value/JOYSTICK_SCALE))/2.0

    def on_L2_release(self):
        # Reset values
        self.tz_demo_1 = 0.0

    # ----------------------------------------
    # Continuous value with Right Thumbstick (UP/DOWN)

    def on_R3_up(self, value):
        # Inverting y-axis value
        self.tz_demo_2 = -1.0 * value/JOYSTICK_SCALE

    def on_R3_down(self, value):
        # Inverting y-axis value
        self.tz_demo_2 = -1.0 * value/JOYSTICK_SCALE

    def on_R3_y_at_rest(self):
        self.tz_demo_2 = 0.0

    # ----------------------------------------
    # Integer tz_demo_3s

    def on_R1_press(self):
        print("R1 button pressed!")
        self.tz_demo_3 += 1

    def on_R1_release(self):
        pass

    def on_L1_press(self):
        print("L1 button pressed!")
        self.tz_demo_3 -= 1

    def on_L1_release(self):
        pass

    # ----------------------------------------

    def on_options_press(self):
        print("Exiting PS4 controller thread.")
        sys.exit()

if __name__ == "__main__":

    # Starting a separate thread for the BT-Controller <<>> RPi communication
    # Press "Options" button to exit this thread

    rob311_bt_controller = ROB311BTController(interface="/dev/input/js0")
    rob311_bt_controller_thread = threading.Thread(target=rob311_bt_controller.listen, args=(10,))
    rob311_bt_controller_thread.start()

    # The "rob311_bt_controller" object has Tz attributes that can be called/used anywhere within the main block--these are potential ways to control
    # vertical axis torque using different commands from the PS4 controller
    # You can also create your own variable (e.g. rob311_bt_controller.tz) and use the button commands to create your own torque command using the button presses

    # DEMO 1: Modifying Tz value with Triggers
    # DEMO 2: Modifying Tz value with Right Thumbstick (UP/DOWN)
    # DEMO 3: Modifying Tz value with Shoulder/Bumper Buttons

    for t in SoftRealtimeLoop(dt=DT, report=True):
        print("\n\nTz Demo 1: {}\nTz Demo 2: {}\nTz Demo 3: {}\n\n".format(
            rob311_bt_controller.tz_demo_1,
            rob311_bt_controller.tz_demo_2,
            rob311_bt_controller.tz_demo_3
        ))