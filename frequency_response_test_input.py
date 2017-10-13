
#!/usr/bin/env python

import time
import numpy as np
import matplotlib.pyplot as plt

from scipy.signal import chirp


def main():

    # frequency sweep params
    f0 = 0.05           # initial frequency
    f1 = 2              # end frequency
    mag = 100           # 
    bias = mag          # amount to shift signal vertically
    t1 = 20             # length of freq sweep in seconds
    method = 'linear'   # sweep type (linear, quadratic, or logarithmic)


#    t = np.linspace(0,20,0.05)

#    input = (chirp(t, f0, f1, t1, method) * mag) + bias

    t = np.linspace(0, 10, 5001)
    w = chirp(t, f0=0.05, f1=2.5, t1=10, method='linear') * mag + bias

    print(w)
    plt.plot(t,w)

    plt.show()

if __name__ == "__main__":
    main()

