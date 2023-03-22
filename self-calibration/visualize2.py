import matplotlib.pyplot as plt
import numpy as np


RUN = 6

data = np.loadtxt(f"./data/run-{RUN}.txt") #
smooth = np.loadtxt(f"./data/run-{RUN}-smooth.txt")
seg = np.loadtxt(f"./data/run-{RUN}-segmented.txt")
lr = np.loadtxt(f"./data/run-{RUN}-lr.txt")

fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, sharex=True)

# Original Data
ax1.plot(data, label='original')
ax1.plot(smooth, label='smoothed')
ax1.set(xlabel='Turn Number', ylabel='Depth Estimate',
       title='Self-Calibration Depth Test')
ax1.legend()
ax1.grid()

# LFR Segmented Data
ax2.plot(seg[:,0], label='Left')
ax2.plot(seg[:,1], label='Front')
ax2.plot(seg[:,2], label='Right')
ax2.set(xlabel='Turn Number', ylabel='Depth Estimate',
        title='Segmented Depth Test')
ax2.legend()
ax2.grid()

# LR Data
ax3.plot(lr[:,0], label='Left')
ax3.plot(lr[:,1], label='Right')
ax3.set(xlabel='Turn Number', ylabel='Depth Estimate',
        title='Left / Right Depth Test')
ax3.legend()
ax3.grid()

fig.tight_layout()
fig.savefig(f"./data/run-{RUN}-plot.png")
