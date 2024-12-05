#!/usr/bin/env python3
import numpy as np

# Define the rotation matrix

approach1 = -0.0493034
approach2 = 0.0238117
approach3 = -0.993835

binormal1 = 0.9964 
binormal2 = -0.0205914
binormal3 = -0.0489931

axis1 = -0.0217279
axis2 = -0.999504 
axis3 = -0.0227793

R = np.array([
    [approach1, binormal1, axis1],
    [approach2, binormal2, axis2],
    [approach3, binormal3, axis3]
])

# Compute RPY
roll = np.arctan2(R[2, 1], R[2, 2])
pitch = np.arcsin(-R[2, 0])
yaw = np.arctan2(R[1, 0], R[0, 0])

# Convert to degrees if needed
roll_deg = np.degrees(roll)
pitch_deg = np.degrees(pitch)
yaw_deg = np.degrees(yaw)



print(f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
