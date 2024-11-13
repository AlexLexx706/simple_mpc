# Stupid Implementation of MPC for a Front-Wheel Steering Car

This project demonstrates a simple implementation of a Model Predictive Controller (MPC) applied to a car model with front-wheel steering. The car is represented in a 2D space, and the MPC controller optimizes the steering angles to follow a predefined path.

The simulation is implemented using PyQt5 for rendering the car model and animation.

## Features
- **MPC Control**: The car uses an MPC controller to navigate through a path by optimizing its steering angles.
- **Ackermann Steering Model**: The car model uses the Ackermann steering geometry for realistic front-wheel steering.
- **Path Following**: You can define start and end points (A and B), and the car will try to follow the shortest path between them.
- **Interactive View**: The PyQt5 view allows you to zoom and pan around the scene.
- **Car Dynamics**: The car model includes parameters like speed, steering angle, car length, and wheel dimensions.

## How to Setup and Run (Linux):
```bash
 python3 -m virtualenv venv
 . ./venv/bin/activate
 pip install -r requirements.txt
 python test_pyqt.py
```