# Implementation of MPC for a Front-Wheel Steering Car with a Trailer in Python using CasADi

This project demonstrates a simple implementation of a Model Predictive Controller (MPC) applied to a car model with front-wheel steering and a trailer. The car is represented in a 2D space, and the MPC controller optimizes the steering angles to follow a predefined path.

The simulation is implemented using PyQt5 for rendering the car model and animation.

## Features
- **MPC Control**: The car uses an MPC controller to navigate through a path by optimizing its steering angles.
- **Path Following**: You can define a path, and the car will attempt to follow it.
- **Interactive View**: The PyQt5 interface allows you to zoom and pan around the scene and interact with MPC settings, such as adding obstacles and editing the path.

## How to Setup and Run in Linux:
```bash
 python3 -m virtualenv venv
 ./venv/bin/pip install -e .
 ./venv/bin/mpc_view
```

## How to Setup and Run in Windows:
```bash
 python3 -m virtualenv venv
 ./venv/Scripts/pip install -e .
 ./venv/Scripts/mpc_view
```

# Video:
[Watch the demo video on YouTube](https://www.youtube.com/watch?v=YC3XxSEILbI)