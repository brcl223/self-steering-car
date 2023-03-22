import matplotlib.pyplot as plt
import numpy as np


RUN = 12
NUM_CALIBS = 4

fig, ax = plt.subplots(nrows=NUM_CALIBS)

for i in range(NUM_CALIBS):
	data = np.loadtxt(f"./data/run-{RUN}-calib-{i}.txt") #
	smooth = np.loadtxt(f"./data/run-{RUN}-calib-{i}-smooth.txt")
	ax[i].plot(data, label='original')
	ax[i].plot(smooth, label='smoothed')
	ax[i].set(xlabel='Turn Number', ylabel='Depth Estimate', title=f'Self-Calibration Depth Test (Calibration {i})')
	ax[i].legend()
	ax[i].grid()

plt.tight_layout()
fig.savefig(f"./data/run-{RUN}-plot.png")
