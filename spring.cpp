#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

  // declare variables
  double m, k, x, v, t_max, dt, t, a, x1, x_prev, x_1;
  vector<double> t_list, x_list, v_list, t1_list, x1_list, v1_list;

  // mass, spring constant, initial position and velocity
  m = 1;
  k = 1;
  x = 0;
  v = 1;
  x_1 = 0;

  // simulation time, timestep and prev x
  t_max = 100;
  dt = 0.1;
  x_prev = x - dt * v;

  // Euler integration
  /*for (t = 0; t <= t_max; t = t + dt) {

    // append current state to trajectories
    t_list.push_back(t);
    x_list.push_back(x);
    v_list.push_back(v);

    // calculate new position and velocity
    a = -k * x / m;
    x = x + dt * v;
    v = v + dt * a;
  }*/

  // Verlet integration
  for (t = 0; t <= t_max; t = t + dt) {

    // append current state to trajectories
    t_list.push_back(t);
    x_list.push_back(x);
    v_list.push_back(v);

    // calculate new position and velocity
    a = -k * x / m;
    x_1 = 2 * x - x_prev + dt * dt * a;
    v = 1 / (2 * dt) * (x_1 - x_prev);
    x_prev = x;
    x = x_1;
  }


  // Write the trajectories to file
  ofstream fout;
  fout.open("trajectories.txt");
  if (fout) { // file opened successfully
    for (int i = 0; i < t_list.size(); i = i + 1) {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
    }
  } else { // file did not open successfully
    cout << "Could not open trajectory file for writing" << endl;
  }

  /* The file can be loaded and visualised in Python as follows:

  import numpy as np
  import matplotlib.pyplot as plt
  results = np.loadtxt('trajectories.txt')
  plt.figure(1)
  plt.clf()
  plt.xlabel('time (s)')
  plt.grid()
  plt.plot(results[:, 0], results[:, 1], label='x (m)')
  plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
  plt.legend()
  plt.show()

  */
}
