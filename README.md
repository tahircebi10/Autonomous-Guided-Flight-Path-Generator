# Autonomous Guided Flight Path Generator

This project is designed to automate the approach of unmanned aerial vehicles (UAVs) to a target location during autonomous flight.

## Features

- Connects to UAVs using the MAVLink protocol
- Calculates distances using the Haversine formula
- Computes bearing angles with Atan2
- Calculates optimal flight paths and generates waypoints
- Provides real-time 3D visualization
- Supports SITL simulation

## Directory

Autonomous-Guided-Flight-Path-Generator/
│
├── src/
│   └── auto_path_generator.py
│
├── tests/
│   └── test-waypoint-sitl.waypoints
│
├── .gitignore
├── LICENSE
├── README.md
└── requirements.txt

## Requirements

```bash
pymavlink>=2.4.0
numpy>=1.19.0
matplotlib>=3.3.0


