# DJI Tello properties

# SI units unless specified otherwise

# Benotsmane, R.; Vásárhelyi, J. Towards Optimization of Energy Consumption of Tello Quad-Rotor with Mpc Model Implementation. Energies 2022, 15, 9207. https://doi.org/10.3390/en15239207 

import numpy as np
mass = 0.08
Ixx = 0.0097
Iyy = 0.0097
Izz = 0.017
# inertia matrix as given in https://in.mathworks.com/help/aeroblks/6dofeulerangles.html
inertiaMat = np.diag([Ixx, Iyy, Izz])

rotorDragCoeff = 0.08
rotorLiftCoeff = 1
halfDiag = 0.06

gravity = 9.81

# Physical dimensions of DJI Tello (in meters)
# Based on actual Tello specs: 98mm x 92.5mm x 41mm
drone_length = 0.098   # x-axis (front-back)
drone_width = 0.0925   # y-axis (left-right)  
drone_height = 0.041   # z-axis (up-down)

# Conservative bounding sphere radius (for RRT* planning)
# Use half the diagonal of the cuboid for safety
drone_radius = np.sqrt(drone_length**2 + drone_width**2 + drone_height**2) / 2.0  # ~0.067m

# Half-extents for easier collision checking
drone_half_extents = np.array([drone_length/2, drone_width/2, drone_height/2])

# Rotor position
rpos = np.array([
    [1, 1, 0],
    [-1, -1, 0],
    [1, -1, 0],
    [-1, 1, 0]
])*halfDiag/np.sqrt(2.0)

# linear control input to thrust mapping
#  This is totally guess work for now
#  Assuming, the drone runs at half throttle during hover 
linearThrustToU = mass*gravity*2/4
linearTorqToU = linearThrustToU/rotorLiftCoeff*rotorDragCoeff