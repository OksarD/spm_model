from numpy import sin, cos, floor, ceil, pi
import numpy as np
import time
import os
import csv
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import threading
from typing import Callable
from types import SimpleNamespace
from scipy.spatial.transform import Rotation as R

class trajectory():
    def __init__(self, _t):
        # ypr trajectory
        self.y = []
        self.p = []
        self.r = []
        self.dy = []
        self.dp = []
        self.dr = []
        # quaternion/xyz trajectory
        self.q = []
        self.omega = []

        self.return_ypr = True
        self.t = _t
        self.dt = _t[1] - _t[0]
        self.index = 0
        self.size = len(_t)

    def derive_ypr(self):
        self.dy = np.gradient(self.y, self.t)
        self.dp = np.gradient(self.p, self.t)
        self.dr = np.gradient(self.r, self.t)

    def convert_ypr_to_q(self):
        if len(self.y) == 0 or len(self.p) == 0 or len(self.r) == 0:
            raise Exception("ypr trajecory not present to convert.")
        else:
            self.q = []
            for i in range(self.size):
                r = R.from_euler('zyx', [self.y[i], self.p[i], self.r[i]])
                self.q.append(r.as_quat())
        self.enforce_quaternion_continuity()

    def enforce_quaternion_continuity(self):
        for i in range(1, self.size):
            if np.dot(self.q[i-1], self.q[i]) < 0:
                self.q[i] = -self.q[i]            

    def derive_xyz(self):
        r = R.from_quat(self.q)
        N = len(r)
        self.omega = np.zeros((N, 3))

        # Forward difference for first element
        q_rel = r[1] * r[0].inv()
        self.omega[0] = (q_rel.as_rotvec() / self.dt) #r[0].as_matrix() @ (q_rel.as_rotvec() / self.dt)

        # Central differences for middle elements
        for i in range(1, N-1):
            q_rel_forward = r[i+1] * r[i].inv()
            q_rel_backward = r[i] * r[i-1].inv()
            omega_local = (q_rel_forward.as_rotvec() - q_rel_backward.as_rotvec()) / (2 * self.dt)
            self.omega[i] = omega_local #r[i].as_matrix() @ omega_local

        # Backward difference for last element
        q_rel = r[N-1] * r[N-2].inv()
        #self.omega[N-1] = r[N-1].as_matrix() @ (q_rel.as_rotvec() / self.dt)
        self.omega[N-1] = (q_rel.as_rotvec() / self.dt)

    def __iter__(self):
        return self

    def __next__(self):
        i = self.index
        self.index += 1
        if i < self.size:
            return SimpleNamespace(q=self.q[i], omega=self.omega[i])
        else:
            raise StopIteration

    def plot(self):
        plt.figure(figsize=(10,6))
        plt.subplot(2,1,1)
        plt.plot(self.t, self.q, label=("x", "y", "z", "w"))
        plt.plot(self.t, self.omega[:,0], label="dx")
        plt.plot(self.t, self.omega[:,1], label="dy")
        plt.plot(self.t, self.omega[:,2], label="dz")
        plt.legend(); plt.grid()
        plt.show(block=False)
        plt.pause(0.001)

class trajectoryGenerator():
    def __init__(self, sample_freq, filter_freq):
        self._sample_freq = sample_freq
        self._filter_freq = filter_freq

    def get_linspace(self, duration):
        num_samples = int(floor(duration*self._sample_freq)) # round down duration so the sample period is maintained
        return np.linspace(0, num_samples/self._sample_freq, num_samples, endpoint=False)

    def filter(self, func):
        b, a = butter(N=1, Wn=self._filter_freq, fs=self._sample_freq, btype='low')
        return filtfilt(b, a, func)

    def sin_func(self, amplitude, period, period_offset=0):
        if period_offset < 0 or period_offset >= 1:
            raise Exception("Period Offset should be in the range 0 <= offset < 1.")
        return lambda t: amplitude*sin((2*pi/period)*(t + period*period_offset))
        
    def cos_func(self, amplitude, period, period_offset=0):
        if period_offset < 0 or period_offset >= 1:
            raise Exception("Period Offset should be in the range 0 <= offset < 1.")
        return lambda t: amplitude*cos((2*pi/period)*((t + period*period_offset)))

    def triangle_func(self, amplitude, period, period_offset=0):
        if period_offset < 0 or period_offset >= 1:
            raise Exception("Period Offset should be in the range 0 <= offset < 1.")
        return lambda t: amplitude*(2*abs(2*((((t + period*period_offset)+period/4)/period)-floor((((t + period*period_offset)+period/4)/period)+0.5)))-1)

    def zero_func(self, *args, **kwargs): # allow additional arguments so that the functions can be swapped easily
        args = args
        kwargs = kwargs
        return lambda t: 0*t

    def trajectory_from_ypr_func(self, duration, y_func, p_func, r_func, filter=True):
        t = self.get_linspace(duration)
        traj = trajectory(t)
        if filter:
            traj.y = self.filter(y_func(t))
            traj.p = self.filter(p_func(t))
            traj.r = self.filter(r_func(t))
        else:
            traj.y = y_func(t)
            traj.p = p_func(t)
            traj.r = r_func(t)
        
        traj.derive_ypr()
        return traj

class loopTimer():
    def __init__(self, interval, callback: Callable, args=[], kwargs={}):
        self._interval = interval
        self._callback = callback
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._args = args
        self._kwargs = kwargs
        self.thread = threading.Thread(target=self.timer_loop, args=(self._stop_event, self._callback, self._args, self._kwargs), daemon=True)

    def start(self):
        self.started = True
        self.thread.start()

    def stop(self):
        self._stop_event.set()

    def pause(self):
        self._pause_event.set()

    def resume(self):
        self._pause_event.clear()

    def timer_loop(self, stop_event, callback: Callable, args, kwargs):
        next_call = time.perf_counter()
        while not stop_event.is_set():
            while self._pause_event.is_set():
                if stop_event.is_set():
                    return
                time.sleep(0.001)
            # execute callback
            callback(*args, **kwargs)
            # Schedule next iteration
            next_call += self._interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're behind schedule, skip sleeping
                next_call = time.perf_counter()
        #print("loop Timer finished")
