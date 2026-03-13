# Pharmacy Inventory Assistant Robot

A ROS2-based simulation project that demonstrates autonomous robot navigation, order-based item retrieval, human-aware motion prediction, and performance benchmarking in a pharmacy-style inventory environment.

---

## Project Overview

This project simulates an autonomous robot that assists in a pharmacy or warehouse-style inventory system. The robot receives item orders, navigates through the environment, retrieves items from predefined locations, and returns to a delivery point.

The system focuses on evaluating robot navigation behavior, interaction with dynamic human movement, and system performance through benchmarking metrics.

---

## Main Capabilities

### Autonomous Navigation
The robot navigates through a simulated environment using goal-based navigation to reach item storage locations and delivery points.

### Order-based Task Execution
Users can input item orders through a command-line interface. The robot processes the order and moves to the corresponding location.

### Object Retrieval Workflow (Simulated Pick-and-Place)
The robot simulates a pick-and-place workflow:
1. Navigate to the storage location  
2. Simulate object pickup  
3. Transport the item  
4. Deliver the item to the destination

### Human Simulation and Prediction
The system includes simulated human movement within the environment. A prediction module estimates human trajectories to allow future integration of human-aware robot behavior.

### Performance Benchmarking
The system records performance metrics such as:
- task completion time
- navigation efficiency
- system execution metrics

These metrics allow evaluation of robotic system performance under different scenarios.

### Environment Visualization
A viewer module visualizes the robot, humans, and environment to observe the robot’s movement and system behavior during simulation.

---

## System Architecture

The project is implemented using modular Python components structured similarly to a ROS2 node architecture.

Key modules include:

- world_node.py – manages the simulated environment  
- goal_nav.py – robot navigation and goal planning  
- human_sim.py – simulates human movement in the environment  
- human_predictor.py – predicts human trajectories  
- order_cli.py – command-line interface for item orders  
- metrics_logger.py – logs system performance metrics  
- viewer.py – visualization of the robot and environment

---

## Technologies Used

- ROS2-based project structure  
- Python  
- Robotics simulation environment  
- Modular robotics software architecture  
- Performance monitoring and logging

---

## Example Workflow

1. User enters an item order using the CLI  
2. Robot receives the order request  
3. Robot navigates to the item location  
4. Robot simulates item retrieval  
5. Robot returns to the delivery point  
6. Performance metrics are recorded

---

## Folder Structure

robot2d_ws/

src/  
└── world_sim/  
    ├── world_node.py  
    ├── goal_nav.py  
    ├── human_sim.py  
    ├── human_predictor.py  
    ├── metrics_logger.py  
    ├── order_cli.py  
    └── viewer.py  

run_world.sh  
run_goal.sh  
run_viewer.sh

---

## Future Improvements

- Integration with real robotic manipulators  
- Vision-based object detection for item recognition  
- Advanced human-aware navigation strategies  
- Reinforcement learning for navigation optimization  
- Deployment on a physical mobile robot platform

---

## Author

Subash Durai  
M.Sc. Autonomy Technologies  
Friedrich-Alexander-Universität Erlangen-Nürnberg
