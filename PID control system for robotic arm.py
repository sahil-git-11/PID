# Project: PID Control System for Robotic Arm Positioning
# Description: Simulate a robotic arm and implement a PID controller to stabilize its movement.

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# taking inputs for robotic arm parameters


mass = int(input("Enter the mass of the arm : "))
length = int(input("Enter the length of the arm  : "))
damping = int(input("Enter the damping coefficient of the arm  : "))
a = int(input("Enter the kp value : "))
b = int(input("Enter the ki value : "))
c = int(input("Enter the kd value : "))

arm_mass = mass # kg
arm_length = length  # m
arm_damping = damping  # damping coefficient

# PID controller parameters
Kp = a  # Proportional gain
Ki = b   # Integral gain
Kd = c   # Derivative gain

# for the desired position (radians)
setpoint = np.pi / 4

# PID controller state
integral = 0.0
previous_error = 0.0

#Takes state (theta, omega) and control input (torque) as arguments.
#Calculates the rate of change of the state (dtheta_dt, domega_dt) based on the armss dynamics.
def robotic_arm_dynamics(t, y, u):
    theta, omega = y
    torque = u
    dtheta_dt = omega
    domega_dt = (torque - arm_damping * omega) / (arm_mass * arm_length**2)
    return [dtheta_dt, domega_dt]

# PID Controller implementation
def pid_controller(error, dt):
    global integral, previous_error
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output

# simulation parameters
#start , end and step
t_start = 0.0 
t_end = 10.0
t_step = 0.01

# initial conditions for the robotic arm (theta = 0, omega = 0)
y0 = [0.0, 0.0]

# time vector for simulation
times = np.arange(t_start, t_end, t_step)

theta_values = []

# Simulation loop
#iterates over each time step.
#calculates error (difference between setpoint and current position).
#computes control signal using the PID controller.
#uses solve_ivp to simulate the arm's dynamics for one step.
#updates the state (theta, omega) based on the simulation result.
#logs the current position (theta) in theta_values.
#state = y0
for i in range(len(times) - 1):
    t = times[i]
    dt = times[i + 1] - t
    
    # Calculate error
    error = setpoint - state[0]

    # Compute control signal
    control_signal = pid_controller(error, dt)

    # Simulate the system for one step
    sol = solve_ivp(
        robotic_arm_dynamics, [t, t + dt], state, args=(control_signal,), t_eval=[t + dt]
    )
    state = sol.y[:, -1]

    # Log theta values
    theta_values.append(state[0])

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(times[:-1], theta_values, label="Arm Position (theta)", color="b")
plt.axhline(y=setpoint, color="r", linestyle="--", label="Setpoint")
plt.xlabel("Time (s)")
plt.ylabel("Theta (radians)")
plt.title("PID Control of Robotic Arm")
plt.legend()
plt.grid()
plt.show()
