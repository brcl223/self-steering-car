import numpy as np


def check_valley(data, idx):
	score = 0

	if data[idx] < data[idx - 1]: score += 1
	if data[idx] < data[idx - 2]: score += 1
	if data[idx] < data[idx + 1]: score += 1
	if data[idx] < data[idx + 2]: score += 1

	if score >= 4:
		return True
	return False


RUN = 12
NUM_CALIBS = 4

for i in range(NUM_CALIBS):
	smooth = np.loadtxt(f"./data/run-{RUN}-calib-{i}.txt")
	#smooth = np.loadtxt(f"./data/run-{RUN}-calib-{i}-smooth.txt")
	dlen = smooth.shape[0]

	print(f"Stats for calibration {i}")
	for j in range(2, dlen - 2):
		if check_valley(smooth, j):
			print(f"Min Index: {j}")
			print(f"Min Value: {smooth[j]}")

