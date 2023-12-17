import numpy as np
import matplotlib.pyplot as plt

m = 1
k = 1
x = 0
v = 1
r = 1 # Verlet velocity

t_max = 100
dt = 0.1
t_array = np.arange(0, t_max, dt)

### Note ###
### The Verlet critical dt is 2, and does not depend on t_max ###

x_list = []
v_list = []
r_list = []

x_prev = x - dt * v

# Verlet integration
for t in t_array:

    x_list.append(x)
    v_list.append(v)
    r_list.append(r)

    a = -k * x / m
    x_1 = 2 * x - x_prev + (dt**2)*(-k*x /m)
    v = v + dt * a
    r = 1/(2*dt) * (x_1 - x_prev)
    x_prev = x
    x = x_1

x_array = np.array(x_list)
v_array = np.array(v_list)
r_array = np.array(r_list)


# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.plot(t_array, r_array, label='r (m/s)')
plt.legend()
plt.show()