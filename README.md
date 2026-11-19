# Quadrotor Modelling and Control

MATLAB/Simulink coursework for a Parrot Mambo aerial-robotics simulator, including reference generation and plotting scripts.

## Operation demo

[**Watch the operation video on Google Drive**](https://drive.google.com/file/d/199s1xvYWt3SIJxd0tVcgLz9azOk-Mlux/view)

The original MP4 is also hosted on GitHub in the [demo release](https://github.com/yonatantamiru65-tech/quadrotor-modelling-control/releases/tag/demo-v1). Open **Assets** to download `quadrotor-operation-demo.mp4` (28.4 MB).

## Project contents

| File or folder | Purpose |
| --- | --- |
| `ref.m` | Simulation parameters, drone dynamics and control-related setup |
| `REF.slx` | Supplied Simulink model |
| `plot.m`, `PLOTS.m` | Plotting and analysis scripts |
| `M1_MAMBO_SIMULATOR_BE_2025_V2_VOID_V2021b.slx` | Simulator model repackaged from the expanded model contents in the archive |
| `BE-TP_Mobile_Robotics_Aerial_Vehicles_2025_26 (3).pdf` | Course/laboratory reference document |

## Requirements

MATLAB, Simulink and Control System Toolbox are required by the supplied scripts. Other products may be required by the models. The simulator folder name references MATLAB V2021b, but compatibility has not been checked.

## Getting started

Inspect `ref.m` and the laboratory document for setup and model dependencies, then open `REF.slx` from this project directory. The simulator `.slx` was repackaged from the supplied expanded model contents without changing those files; MATLAB loading has not been validated. No scripts have been executed or modified during repository preparation.

## Attribution and scope

This is a coursework collection supplied by Yonatan Tamiru Asfaw, not a claim of sole authorship of the simulator. `ref.m` retains its original author header crediting Pierre Susbielle. Course material, supplied models and student contributions remain distinguishable through their original files and notices. No new software license has been assigned.

Generated MATLAB build caches and autosaves are excluded.
