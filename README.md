# ROADGUARD AI — MATLAB Starter Prototype

**Status:** Educational software prototype / source-code starter.  
**Problem context:** SIH26037 — perception-to-planning intelligence for Indian road conditions.

This repository contains an initial MATLAB implementation of:
- A simple 2-D vehicle simulation loop (`main.m`)
- Grid-based A* path planning (`07_planning/astarPlanner.m`)
- Constant-velocity motion prediction (`05_prediction/motionPredictor.m`)
- Basic trajectory-vs-obstacle collision checking (`06_collision/trajectoryCollision.m`)
- Test scripts for planning, prediction, and collision modules

## Important limitations
This is **not** a complete autonomous-driving system and does not include trained perception models, camera/LiDAR/radar fusion, a Simulink vehicle model, V2V communication, or road-tested safety logic. The example uses synthetic grid data and simple kinematic assumptions. Do not use it to control a real vehicle.

## Requirements
- MATLAB (not validated in every release)
- No toolboxes are required for the starter algorithms

## Run
1. Clone or download this repository.
2. Open MATLAB and set the current folder to `ROADGUARD-AI`.
3. Run `main`.

Run individual checks:
```matlab
run('07_planning/testAstar.m')
run('05_prediction/testPrediction.m')
run('06_collision/testCollision.m')
```

## Folder structure
```text
ROADGUARD-AI/
├── main.m
├── README.md
├── 05_prediction/
│   ├── motionPredictor.m
│   └── testPrediction.m
├── 06_collision/
│   ├── trajectoryCollision.m
│   └── testCollision.m
└── 07_planning/
    ├── astarPlanner.m
    └── testAstar.m
```

## Suggested next steps
1. Replace synthetic obstacle grids with a defined road-map representation.
2. Add unit tests and record MATLAB version / test results.
3. Add perception outputs only when real model/code is available.
4. Integrate validated vehicle dynamics and safety constraints before any hardware work.
