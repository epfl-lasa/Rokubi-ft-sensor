import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, windows, convolve

import rospy
import rospkg
import rosbag

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('rokubi_ft_driver') + '/log/log.bag')

wrenchData = list(bag.read_messages(topics=['/wrench_data']))
print(len(wrenchData))

forceData = np.zeros((3, len(wrenchData)))
torqueData = np.zeros((3, len(wrenchData)))
time = np.zeros(len(wrenchData))

for i, msg in enumerate(wrenchData):

    newForce = msg.message.wrench.force
    newTorque = msg.message.wrench.torque

    forceData[:, i] = np.array([newForce.x, newForce.y, newForce.z ])
    torqueData[:, i] = np.array([newTorque.x, newTorque.y, newTorque.z ])

    time[i] = msg.message.header.stamp.to_sec()



# win = np.matlib.repmat(windows.hann(1), 1, 6).reshape(1, 6)
# wrench2 = savgol_filter(wrench, 31, 3, axis = 0)
# filtered = convolve(wrench, win, mode='same') / sum(win)

# plt.plot(time, forceData.T)
# plt.legend(["Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"])

plt.plot(1000*np.diff(time))

# plt.plot(sensorTime, wrench[:, 0])
# plt.plot(sensorTime, wrench2[:, 0])

plt.show()