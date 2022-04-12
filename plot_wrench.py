import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, windows, convolve

data = np.loadtxt("data.txt")
time = data[:, 0]
wrench = data[:, 1:]
dT = 1.0/800
sensorTime = np.linspace(0, len(data)*dT, len(data))

win = np.matlib.repmat(windows.hann(1), 1, 6).reshape(1, 6)
wrench2 = savgol_filter(wrench, 31, 3, axis = 0)
filtered = convolve(wrench, win, mode='same') / sum(win)

# plt.plot(time, wrench, "-+")
plt.legend(["Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"])

plt.plot(np.diff(time))

# plt.plot(sensorTime, wrench[:, 0])
# plt.plot(sensorTime, wrench2[:, 0])

plt.show()