import numpy as np
import matplotlib.pyplot as plt

results = np.loadtxt('C:\\Users\\noahf\\source\\repos\\lander\\lander\\Altitude_VEr.txt')
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='Altitude (m)')
plt.plot(results[:, 0], results[:, 2], label='Vertical Velocity (m/s)')
plt.legend()
plt.show()