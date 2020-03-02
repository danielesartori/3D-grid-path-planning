# 3D-grid-path-planning

Some Matlab tools for path planning on a 3D grid map, including 3D A star, 3D Theta star and 3D Lazy Theta star

- Main_3D.m: main file for defining the map and the path properties, to call the grid and path generation functions and to plot the results

- Grid_3D_safe_zone.m: randomly generates a 3D cluttered environment (obstacles connected to the groud), which is represented both as a 2D matrix and as 3D occupancy map. For both maps, a safety buffer zone is created around and above obstacles

- a_star_3D.m: 3D A* path planning algorithm

- theta_star_3D.m: 3D Theta* path planning algorithm

- lazy_theta_star_3D.m: 3D Lazy Theta* path planning algorithm

- line_sight_partial_3D.m: line of sight algorithm for Theta* and Lazy Theta*, it includes the check of the crossing within a safety buffer zone

This code was used in the paper:
D. Sartori, D. Zou, W. Yu, An efficient approach to near-optimal 3D trajectory design in cluttered environments for multirotor UAVs, 2019 IEEE 15th International Conference on Automation Science and Engineering (CASE), Vancouver, August 2019
Available at:
https://ieeexplore.ieee.org/abstract/document/8842980/
