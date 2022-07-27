import numpy as np
import matplotlib.pyplot as plt

import rospy
import rospkg
import rosbag


rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('rokubi_ft_driver') + '/log/log.bag')

wrenchData = list(bag.read_messages(topics=['/wrench_data']))

forceData = np.zeros((3, len(wrenchData)))
torqueData = np.zeros((3, len(wrenchData)))
time = np.zeros(len(wrenchData))

for i, msg in enumerate(wrenchData):

    newForce = msg.message.wrench.force
    newTorque = msg.message.wrench.torque

    forceData[:, i] = np.array([newForce.x, newForce.y, newForce.z ])
    torqueData[:, i] = np.array([newTorque.x, newTorque.y, newTorque.z ])

    time[i] = msg.message.header.stamp.to_sec()

time = time - time[0]

for i in range(3):
    print(np.mean(forceData[i, :]))
    print(np.mean(torqueData[i, :]))


plt.subplot(2, 1, 1)
plt.plot(time, forceData.T)
plt.legend(["Force X", "Force Y", "Force Z"])

plt.subplot(2, 1, 2)
plt.plot(time, torqueData.T)
plt.legend(["Torque X", "Torque Y", "Torque Z"])


plt.show()