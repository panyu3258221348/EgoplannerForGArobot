# Terrestrial-Aerial-Navigation

**Terrestrial-Aerial-Navigation** is an autonomous navigation framework that brings complete autonomy to terrestrial-aerial bimodal vehicles (TABVs). This repository contains the following sub-modules:

- A bi-level motion planner which generates safe, smooth, and dynamically feasible terrestrial-aerial hybrid trajectories.
- A customized [TABV platform](https://github.com/ZJU-FAST-Lab/TABV-Platform) that carries adequate sensing and computing resources while ensuring portability and maneuverability.

> **Update**: The planner has been migrated from [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) to [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner), removing CUDA, NLopt, and ESDF dependencies. Planning time ~1ms.

## About

If our source code or hardware platform is used in your academic projects, please cite the related papers below.

- [Autonomous and Adaptive Navigation for Terrestrial-Aerial Bimodal Vehicles](https://ieeexplore.ieee.org/document/9691888), Ruibin Zhang, Yuze Wu, Lixian Zhang, Chao Xu, and Fei Gao, IEEE Robotics and Automation Letters (**RA-L**), 2022.

```
@ARTICLE{Zhang2022TABV,
      author={Zhang, Ruibin and Wu, Yuze and Zhang, Lixian and Xu, Chao and Gao, Fei},
      journal={IEEE Robotics and Automation Letters}, 
      title={Autonomous and Adaptive Navigation for Terrestrial-Aerial Bimodal Vehicles}, 
      year={2022},
      volume={7},
      number={2},
      pages={3008-3015}
}
```

- [EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors](https://ieeexplore.ieee.org/document/9345365), Xin Zhou, Zhepei Wang, Chao Xu, Fei Gao, IEEE Robotics and Automation Letters (**RA-L**), 2020.

```
@ARTICLE{Zhou2020EGO,
      author={Zhou, Xin and Wang, Zhepei and Xu, Chao and Gao, Fei},
      journal={IEEE Robotics and Automation Letters}, 
      title={EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors}, 
      year={2020},
}
```

Video Links: [TABV Youtube](https://www.youtube.com/watch?v=Bdb5mK9OKIo), [EGO-Planner Youtube](https://www.youtube.com/watch?v=GK3cg5dR6lU).

<p align="center">
  <img src="figs/cover.png" width="600"/>
</p>

## Architecture

```
src/
├── TIE_navigation/          # Core planning framework
│   ├── plan_env/            #   Occupancy grid map (GridMap, no ESDF)
│   ├── path_searching/      #   A* front-end path search
│   ├── bspline_opt/         #   L-BFGS + rebound trajectory optimization
│   ├── bspline/             #   B-spline curve utilities
│   ├── traj_utils/          #   Trajectory visualization & polynomial traj
│   └── plan_manage/         #   EGOReplanFSM, launch files, RViz config
└── uav_simulator/           # Lightweight simulation
    ├── fake_drone/          #   Quadrotor dynamics simulation
    ├── local_sensing/       #   Point cloud renderer (CPU only)
    ├── map_generator/       #   Random obstacle forest generator
    └── Utils/               #   Shared utilities & RViz plugins
```

## Quick Start

Tested on **Ubuntu 20.04 + ROS Noetic**.

```bash
sudo apt-get install libarmadillo-dev

git clone https://github.com/ZJU-FAST-Lab/Terrestrial-Aerial-Navigation.git
cd Terrestrial-Aerial-Navigation
catkin_make -j$(nproc)
source devel/setup.bash
roslaunch plan_manage ego_replan.launch
```

Use the **2D Nav Goal** tool in RViz to select a goal. The drone will plan and execute collision-free trajectories.

<p align="center">
<img src="figs/sim.gif" width="700" height="400" border="3" />
</p>

## Launch Files

| Launch File | Description |
|---|---|
| `ego_replan.launch` | **Main** — EGO-Planner with simulation, RViz |
| `kino_replan.launch` | Fast-Planner with kinodynamic A* (backup) |

## Configuration

Key parameters in `ego_replan.launch`:

| Param | Default | Description |
|---|---|---|
| `map_size_x/y/z` | 20/20/5 | Map dimensions (meters) |
| `max_vel` | 2.0 | Max velocity (m/s) |
| `max_acc` | 3.0 | Max acceleration (m/s²) |
| `planning_horizon` | 7.5 | Planning horizon (meters) |
| `flight_type` | 1 | 1=manual 2D Nav Goal, 2=preset waypoints |
| `p_num` | 80 | Number of random obstacles |

## Key Differences from Fast-Planner

| Component | Fast-Planner (old) | EGO-Planner (current) |
|---|---|---|
| Mapping | SDF + ESDF distance field | Occupancy grid (no ESDF) |
| Front-end | Kinodynamic A* | Standard A* |
| Optimization | NLopt gradient-based | L-BFGS + rebound |
| Planning time | ~10ms | ~1ms |
| Dependencies | NLopt, CUDA (optional) | None extra |

## Acknowledgements

Built on [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) and [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner). Uses [LBFGS-lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) for numerical optimization.

## Licence

The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

## Maintenance

For technical issues, please contact Ruibin Zhang (ruibin_zhang@zju.edu.cn) or Fei Gao (fgaoaa@zju.edu.cn).

For commercial inquiries, please contact Fei Gao (fgaoaa@zju.edu.cn).
