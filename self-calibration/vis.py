import matplotlib.pyplot as plt
import numpy as np

def smooth_data(data):
    data = np.array(data)
    dlen = data.shape[0]

    smooth_data = np.zeros(data.shape)
    conv_filt = [0.25, 0.5, 0.25]

    smooth_data[0] = np.dot(conv_filt, [data[0], data[0], data[1]])
    for i in range(1, dlen - 1):
        smooth_data[i] = np.dot(conv_filt, data[i-1:i+2])
    smooth_data[-1] = np.dot(conv_filt, [data[-2], data[-1], data[-1]])

    return smooth_data

x = np.linspace(-np.pi, np.pi, 201)
y = np.sin(x) + np.sin(2*x) + 2*np.cos(x)
y2 = smooth_data(y)
y3 = smooth_data(y2)
y4 = smooth_data(y3)
for j in range(100):
    y4 = smooth_data(y4)

plt.plot(x, y, label='sin(x)')
plt.plot(x, y2, label='smooth(sin(x))')
plt.plot(x, y3, label='smooth^2(sin(x))')
plt.plot(x, y4, label='smooth^100(sin(x))')
plt.legend()
plt.axis('tight')
plt.show()
