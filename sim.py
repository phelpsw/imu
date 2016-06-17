#!/usr/bin/env python

import numpy as np

vel_trg = 134.112
accel_dist = 914.0
decel_dist = 304.8
accel_dt = 2.0 * accel_dist / vel_trg
decel_dt = 2.0 * decel_dist / vel_trg
accel = vel_trg / accel_dt
decel = -vel_trg / decel_dt
start_offset_t = 10.0
start_t = start_offset_t + accel_dt
stop_t = start_t + decel_dt
sim_end_t = stop_t + 10.0
sensor_orientation = np.eye(3)

stationary_noise_mag = 0.1
moving_noise_mag = 4.0
gravity = 9.81

# In vehicle body frame
accel_profile = [(0.0, (0.,0.,0.)),
                 (start_offset_t, (accel,0.,0.)),
                 (start_t, (decel,0.,0.)),
                 (stop_t, (0.,0.,0.)),
                 (sim_end_t, (0.,0.,0.))]


# Simulation step size (sec)
#dt = 1/1600.0
dt = 1.
t = 0.0
event_index = 0
ts_data = []

# Vehicle Reference Frame
vehicle_accel = []
vehicle_vel = []
vehicle_pos = []

# Sensor Reference Frame
accel_data = []
vel_data = []
pos_data = []

while t < accel_profile[len(accel_profile) - 1][0]:
    if t > accel_profile[event_index + 1][0]:
        event_index += 1
        continue
    ts_data.append(t)

    # Build measured accel vector

    mag = np.linalg.norm(accel_profile[event_index][1])
    accel_data.append(accel_profile[event_index][1] + (mag,))

    






    t += dt







import matplotlib.pyplot as plt
fig, axs = plt.subplots(2, 2)

# linear
ax = axs[0, 0]
for l in zip(*accel_data):
    ax.plot(ts_data, list(l))
ax.set_title('Accel')
ax.grid(True)

plt.show()



