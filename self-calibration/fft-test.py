import matplotlib.pyplot as plt
import numpy as np


def decompose_fft(data, threshold = 0.5):
    fft3 = np.fft.fft(data)
    # x = np.arange(0, 10, 10 / len(data))
    # x = np.arange(0, data.shape[0] * 2)
    x = np.arange(0, data.shape[0], data.shape[0] / 10000)
    x2 = np.arange(0, data.shape[0])
    # freqs = np.fft.fftfreq(data.shape[-1], .01)
    freqs = np.fft.fftfreq(data.shape[0], 1)
    recomb = np.zeros(x.shape[0])
    for i in range(len(fft3)):
        if abs(fft3[i]) / data.shape[0] > threshold:
            sinewave = (
                1 / data.shape[0] * (fft3[i].real * np.cos(freqs[i] * 2 * np.pi * x)
                    - fft3[i].imag * np.sin(freqs[i] * 2 * np.pi * x)))
            recomb += sinewave
            print(recomb)
            plt.plot(x, sinewave)
    plt.show()

    plt.plot(x, recomb, label='recombined')
    plt.plot(x2, data, label='OG')
    plt.legend()
    plt.show()


def main():
    data = np.loadtxt("./data/run-1.txt")

    fig = plt.figure()
    ax = plt.axes(projection="3d")

    fft_data = np.fft.fft(data)
    ifft_data = np.fft.ifft(fft_data)

    dlen = int(data.shape[0] / 2)
    # ax.scatter(np.arange(dlen), np.imag(fft_data[0:dlen]), np.real(fft_data[0:dlen]))
    ax.scatter(np.arange(dlen), np.imag(ifft_data[0:dlen]), np.real(ifft_data[0:dlen]))
    # fig.savefig("./data/fft-test.png")
    plt.show()


if __name__ == '__main__':
    data = np.loadtxt("./data/run-1.txt")
    data /= 1e8
    decompose_fft(data)

    # sp   = np.fft.fft(data)               # the discrete fourier transform
    # freq = np.fft.fftfreq(data.shape[-1]) # the accompanying frequencies

    # cos=np.sum([(sp[-i]+sp[i]).real/(2*T)*np.cos(2.*np.pi*freq[i]*x) for i in range(len(freq))],axis=0)
    # sin=np.sum([(sp[-i]-sp[i]).imag/200.*np.sin(2.*np.pi*freq[i]*x) for i in range(len(freq))],axis=0)

    # plt.plot(x, y,x,cos+sin)
    # plt.show()
