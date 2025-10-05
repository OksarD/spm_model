from numpy import sin, cos, floor, ceil, pi
import numpy as np
import time
import os
import csv
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import threading
from typing import Callable

class trajectory():
    def __init__(self, func, deriv, t):
        self._func = func
        self._deriv = deriv
        self._t = t
        self.end = len(t)
        self.index = 0
        self.is_zero = False
        self.size = len(t)

    def __iter__(self):
        return self

    def __next__(self):
        i = self.index
        self.index += 1
        if i < self.end:
            return (self._func[i],
                    self._deriv[i],
                    self._t[i])
        else:
            raise StopIteration

    def plot(self):
        plt.figure(figsize=(10,6))
        plt.subplot(2,1,1)
        plt.plot(self._t, self._func, label="func")
        plt.plot(self._t, self._deriv, label="deriv")
        plt.legend(); plt.grid()
        plt.show(block=False)
        plt.pause(0.001)

class pathGenerator():
    def __init__(self, sample_freq, filter_freq):
        self._sample_freq = sample_freq
        self._filter_freq = filter_freq

    def get_linspace(self, duration):
        num_samples = int(floor(duration*self._sample_freq)) # round down duration so the sample period is maintained
        return np.linspace(0, num_samples/self._sample_freq, num_samples, endpoint=False)

    def filter(self, func):
        b, a = butter(N=1, Wn=self._filter_freq, fs=self._sample_freq, btype='low')
        return filtfilt(b, a, func)

    def generate_sin_trajectory(self, amplitude, period, duration, filter=False):
        t = self.get_linspace(duration)
        wave = amplitude*sin((2*pi/period)*t)
        if filter == True:
            wave = self.filter(wave)
        wave_deriv = np.gradient(wave, t)
        return trajectory(wave, wave_deriv, t)

    def generate_cos_trajectory(self, amplitude, period, duration, filter=False):
        t = self.get_linspace(duration)
        wave = amplitude*cos((2*pi/period)*t)
        if filter == True:
            wave = self.filter(wave)
        wave_deriv = np.gradient(wave, t)
        return trajectory(wave, wave_deriv, t)

    def generate_triangle_trajectory(self, amplitude, period, duration, filter=False):
        t = self.get_linspace(duration)
        wave = amplitude*(2*abs(2*(((t+period/4)/period)-floor(((t+period/4)/period)+0.5)))-1)
        if filter == True:
            wave = self.filter(wave)
        wave_deriv = np.gradient(wave, t)
        return trajectory(wave, wave_deriv, t)
    
    def generate_zero_trajectory(self, duration):
        t = self.get_linspace(duration)
        zero_func = 0*t
        traj = trajectory(zero_func, zero_func, t)
        traj.is_zero = True
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
