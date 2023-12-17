// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
vector3d drag;

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    double altitude, e, v_er, K_h, K_p, Delta, P_out;

    // Define K_h, K_p and Delta
    K_h = 0.013;
    K_p = 0.5;
    Delta = 0.24;

    // Calculate altitude and vertical velocity
    altitude = position.abs() - MARS_RADIUS;
    v_er = velocity * position.norm();
    
    // Calculate error and P_out
    e = -(0.5 + K_h * altitude + v_er);
    P_out = K_p * e;

    //orientation
    //orientation = vector3d()

    // Start with normal throttle
    throttle = P_out + Delta;

    if ((altitude <= 30000) && (velocity.abs() < MAX_PARACHUTE_SPEED) && (drag.abs() < MAX_PARACHUTE_DRAG)) {
        parachute_status = DEPLOYED;
    }

    if (P_out + Delta >= 1) {
        throttle = 1;
    }
    else if (P_out + Delta <= 0) {
        throttle = 0;
    }
        
}

void numerical_dynamics(void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
    vector3d net_force, thr, gravity, next_position, acceleration;
    static vector3d previous_position;
    double mass;

    // Calculate mass of the lander
    mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;

    // Calculate forces acting on the shuttle
    thr = thrust_wrt_world();
    drag = - 0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * (3.1415 * LANDER_SIZE * LANDER_SIZE) * velocity.abs() * velocity; // Here abs is not sqwuared as velocity is not normalized.
    if (parachute_status == DEPLOYED) {
        drag += -0.5 * atmospheric_density(position) * DRAG_COEF_CHUTE * 5.0 * (4.0 * LANDER_SIZE * LANDER_SIZE) * velocity.abs() * velocity;
    }
    gravity = - GRAVITY * MARS_MASS * mass / (position.abs2() * position.abs()) * position;
    
    // Net force and acceleration
    net_force = thr + drag + gravity;
    acceleration = net_force / mass;

    /*
    // Euler integration
    velocity += delta_t * acceleration;
    position += delta_t * velocity;
    */
    
    // Verlet integration
    if (simulation_time <= delta_t) {
        next_position = position + delta_t * velocity;
    }
    else {
        next_position = 2 * position - previous_position + (delta_t * delta_t) * acceleration;
        velocity = 1 / (2 * delta_t) * (next_position - previous_position);
    }

    previous_position = position;
    position = next_position;
    

    /*
    // Calculate altitude and vertical velocity
    altitude = position.abs() - MARS_RADIUS;
    v_er = velocity * position.norm();
    // Record vertical velocity and altitude at each timestep
    ofstream fout;
    fout.open("Altitude_VEr.txt", ios_base::app);
    if (fout) {
        fout << simulation_time << ' ' << altitude << ' ' << v_er << ' ' << endl;
    }
    else {
        cout << "Could not open file" << endl;
    }*/
    

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
