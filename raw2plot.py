#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np

decision = np.fromfile("dc.raw", dtype=np.float32)
f_decision = np.fromfile("f_dc.raw", dtype=np.float32)
v0=np.fromfile('v0.raw', dtype=np.float32)
v1=np.fromfile('v1.raw', dtype=np.float32)

plt.title("v0 (red) and v1 (green)")
plt.plot(v0, 'r')
plt.plot(v1, 'g')
plt.show()

plt.title("decision (red) and filtered decision (green)")
plt.plot(decision, 'r')
plt.plot(f_decision, 'g')
plt.show()