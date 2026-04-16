import smbus2
import numpy as np
class XelaTactileDriver:
def __init__(self, i2c_bus=1, address=0x40):
self.bus = smbus2.SMBus(i2c_bus)
self.address = address
self.num_taxels = 16
def read_frame(self):
"""Read capacitance values from all taxels"""
data = []
for i in range(self.num_taxels * 2):
try:
val = self.bus.read_word_data(self.address, i)
data.append(val)
except IOError:
data.append(0)
# Convert to capacitance values
capacitance = np.array(data, dtype=np.float32)
capacitance = capacitance.reshape(self.num_taxels, 2)
return capacitance[:, 0] + 1j * capacitance[:, 1]
def calibrate(self, num_samples=100):
"""Calculate baseline capacitance"""
samples = []
for _ in range(num_samples):
samples.append(self.read_frame())
self.baseline = np.mean(samples, axis=0)
return self.baseline
23
def get_pressure(self):
"""Compute pressure from capacitance change"""
current = self.read_frame()
delta = np.abs(current - self.baseline)
# Linear approximation: pressure ˜ capacitance change
pressure = delta * self.calibration_factor
return pressure
