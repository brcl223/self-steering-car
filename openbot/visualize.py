import numpy as np

def smooth_data(data):
    data = np.array(data)
    dlen = data.shape[0]

    if dlen < 2:
        return data

    smooth_data = np.zeros(data.shape)
    conv_filt = [1/4, 1/2, 1/4]

    smooth_data[0] = np.dot(conv_filt, [data[0], data[0], data[1]])

    for i in range(1, dlen - 1):
        smooth_data[i] = np.dot(conv_filt, data[i-1:i+2])

    smooth_data[-1] = np.dot(conv_filt, [data[-2], data[-1], data[-1]])

    return smooth_data

x = np.linspace(-np.pi, np.pi, 201)
y1 = np.sin(x)
y2 = smooth_data(y1)
y3 = smooth_data(y2)

import matplotlib.pyplot as plt
plt.plot(x, y1, label='sin(x)')
plt.plot(x, y2, label='smooth(sin(x))')
plt.plot(x, y3, label='smooth^2(sin(x))')
plt.legend()
plt.axis('tight')
plt.show()
