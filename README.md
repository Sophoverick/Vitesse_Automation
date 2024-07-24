# Simulation
This file as whole contains the scripts of Team Vitesse's pymavlink code in a ros wrapper specialized for simulation, the implementation code is not open for release, the drone is automated to catch other drones.The saved_model.pt file is an unoptimized neural network of YOLO v5n for detecting the drones in simulation environment.
## Simulation with gazebo
## Simulation without gazebo
## AI functions
This library contains the triangulation function, the function that calculates the distance to target drone by ascending and descending and calculating distance via the angles from both places.
# dronefunctions
Drone functions is a library filled with simplified pymavlink functions, instead of having to write long commands, it is possible to simply go forward and yaw to the target drone instead of going to specific local coordinates. It also contains functions for yawing to a location or going to a relative height 
