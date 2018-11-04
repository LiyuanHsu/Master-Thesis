# Project Title

Transfer Learning for Mapless Quadrotor Navigation Using Recurrent Neural Network

## Getting Started

There are two parts for this project
* Collecting the state-action pairs from a motion planner
* Training Reinforcement learning navigation model in OpenAI Gym environment 

## Prerequisites

* [flatland](https://github.com/avidbots/flatland) - Ros depository for map and robot setup for 2D flatland navigation environment
* [turtlebot_flatland](https://github.com/avidbots/turtlebot_flatland) - Turtlebot navigation demo under flatland
* [OpenAI/Gym](https://github.com/openai/gym) - A toolkit for developing and comparing reinforcement learning algorithms

### Others

* [bebop_vicon_ctrl](https://github.com/mdeyo/bebop_vicon_control) - Ros setup for control bebop in vicon envionemnt


## Repositories
* globalplan_record: Code for record the navigation actions from ros-navigation planner
* lidar: connection and plotting LIDAR distance readings

## Map naming

### Map for turtleble_flatland
```
turtlebot_flatland/maps
```

* Cyliner_map -> cylinder_map1.png 
* Rectangle_map -> rectangle_map1.png 
* Wall_map -> wall_map1.png 
* Office_map -> complex_map3.png 
* Street_map -> complex_map5_2.png 
* Forest_map -> complex_map6.png

### Map for OpenAI Gym
```
gym-drone_environments/gym/envs/curriculum/envs
```

* Cyliner_map -> planner_cylinder_map4.py
* Rectangle_map -> planner_rectangle_map1.py
* Wall_map -> planner_wall_map1.py
* Cylinder_map + Rectangle_map -> planner_cylinder4_rectangle1.py
* Cylinder_map + Wall_map -> planner_cylinder4_wall1.py
* Rectangle_map + Wall_map -> planner_rectangle1_wall1.py
* Cylinder_map + Rectangle_map + Wall_map -> planner_cylinder4_rectangle1_wall1.py
* Office_map -> planner_complex_map4.py
* Street_map -> planner_complex_map5_1.py
* Forest_map -> planner_complex_map6.py

## Acknowledgments

* Stefan Stevsic, ETH Zürich
* Prof. Dr. Otmar Hilliges, ETH Zürich
* Prof. Dr. Moritz Diehl, University of Freiburg
