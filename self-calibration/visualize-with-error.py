import matplotlib.pyplot as plt
import numpy as np


RUN = 8
data = np.loadtxt(f"./data/run-{RUN}-calib-2.txt") #
smooth = np.loadtxt(f"./data/run-{RUN}-calib-2-smooth.txt")

# Percentage
yerr = 5
xerr = 0

fig, ax = plt.subplots()
ax.plot(data, label='original')
ax.plot(smooth, label='smoothed')
ax.set(xlabel='Turn Number', ylabel='Depth Estimate',
       title='Self-Calibration Depth Test')
ax.legend()
ax.errorbar(np.arange(len(data)), data, yerr=data*yerr/100., color='black', barsabove='False')
ax.errorbar(np.arange(len(smooth)), smooth, yerr=smooth*yerr/100., color='black', barsabove='False')
ax.grid()

fig.savefig(f"./data/run-{RUN}-plot-error.png")
