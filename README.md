EE 470 Final Project — Lateral Controller Comparison in CARLA

**Authors:** Preston Kearnan & Ches Zepp

## Overview
Implements and compares PID and Pure Pursuit lateral controllers for autonomous
vehicle path tracking in the CARLA simulator (Town03)

## Requirements
- CARLA 0.9.16
- Python 3.12
- numpy, matplotlib

## Setup
1. Launch `CarlaUE4.exe`
2. Install the CARLA Python `.whl` package from your CARLA installation
3. `pip install numpy matplotlib`

## Usage
```bash
python finalproject.py
```
Switch between controllers by setting `CONTROLLER = "PID"` or `CONTROLLER = "PP"` at the top of the script.

## Test Routes
Set `SPAWN_INDEX` and `GOAL_WAYPOINT` in the script to select a route:
- "Hook" - 9 and 91
- "Half Roundabout" - 0 and 24
- "Hills" - 46 adn 52
