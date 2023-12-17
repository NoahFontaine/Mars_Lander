import numpy as np
import matplotlib.pyplot as plt

m = 5
M = 6.42e23
G = 6.6743e-11
r = 3389000

disp = np.array([0,0,3389001])
v = np.array([100,0,3000])

# simulation time, timestep and time
t_max = 10000
dt = 10
t_array = np.arange(0, t_max, dt)

# functions that draw the planet
p = np.linspace(-r, r, 2*3389 + 1)
def f(x):
  return np.sqrt(r**2 - x**2)
def f1(x):
  return -np.sqrt(r**2 - x**2)

# initialise empty lists to record trajectories
disp_list = []
v_list = []

# Euler integration
for t in t_array:

  # append current state to trajectories
  if np.linalg.norm(disp) <= r:
    disp_list.append(np.array([disp[0],disp[1],disp[2]]))
    v_list.append(np.array([0,0,0]))
  else:
    disp_list.append(disp)
    v_list.append(v)

    # calculate new position and velocity
    a = -G * M / (np.linalg.norm(disp))**3 * disp
    disp = disp + dt * v
    v = v + dt * a

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
disp_array = np.array(disp_list)
v_array = np.array(v_list)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, disp_array[:,2], label='x (m)')
plt.legend()
plt.show()

plt.figure(2)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, v_array[:,2], label='v (m/s)')
plt.legend()
plt.show()

plt.figure(2)
plt.clf()
plt.xlabel('x position (m)')
plt.grid()
plt.plot(disp_array[:,0], disp_array[:,2], label='altitude (m)')
plt.plot(disp_array[:,0], disp_array[:,1], label='altitude (m)')
plt.plot(p, f(p), label='Mars')
plt.plot(p, f1(p), label='Mars')
plt.legend()
plt.show()

