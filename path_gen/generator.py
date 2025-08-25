from numpy import sin, cos, floor, ceil, pi
import numpy as np
import time
import os
import csv
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

class trajectory():
    def __init__(self, func, deriv, t):
        self._func = func
        self._deriv = deriv
        self._t = t

    def plot(self):
        plt.figure(figsize=(10,6))
        plt.subplot(2,1,1)
        plt.plot(self._t, self._func, label="func")
        plt.plot(self._t, self._deriv, label="deriv")
        plt.legend(); plt.grid()
        plt.show()

class pathGenerator():
    def __init__(self, sample_freq, filter_freq):
        self._sample_freq = sample_freq
        self._filter_freq = filter_freq

    def get_linspace(self, duration):
        num_samples = int(floor(duration*self._sample_freq)) # round down duration so the sample period is maintained
        return np.linspace(0, num_samples/self._sample_freq, num_samples, endpoint=False)

    def get_numerical_derivative(self, func, t):
        dt = t[1] - t[0]
        return np.gradient(func, t)

    def filter(self, func):
        b, a = butter(N=1, Wn=self._filter_freq, fs=self._sample_freq, btype='low')
        return filtfilt(b, a, func)

    def generate_sin_trajectory(self, amplitude, period, duration):
        t = self.get_linspace(self, duration)
        wave = self.filter(amplitude*sin((2*pi/period)*t))
        wave_deriv = self.get_numerical_derivative(wave, t)
        return trajectory(wave, wave_deriv, t)

    def generate_cos_trajectory(self, amplitude, period, duration):
        t = self.get_linspace(duration)
        wave = self.filter(amplitude*cos((2*pi/period)*t))
        wave_deriv = self.get_numerical_derivative(wave, t)
        return trajectory(wave, wave_deriv, t)

    def generate_triangle_trajectory(self, amplitude, period, duration):
        t = self.get_linspace(duration)
        wave = self.filter(amplitude*(2*abs(2*((t/period)-floor((t/period)+0.5)))-1))
        wave_deriv = self.get_numerical_derivative(wave, t)
        return trajectory(wave, wave_deriv, t)
