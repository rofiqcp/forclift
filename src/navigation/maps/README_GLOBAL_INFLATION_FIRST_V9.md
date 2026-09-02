# V9 Global Inflation First

Launch:
```bash
ros2 launch navigation autonomous.launch.py
```

First visual milestone:
1. saved `/map`
2. RobotModel / URDF
3. `/global_costmap/costmap`
4. StaticLayer
5. ObstacleLayer
6. InflationLayer

Only after this milestone:
- RViz opens
- ControllerServer starts
- local costmap / local inflation starts
- BT Navigator / smoother / collision monitor start

Costmap:
- resolution: 0.05 m/cell
- inflation_radius: 0.75 m = 15 cells
- cost_scaling_factor: 3.0

The inflation plugin intentionally remains LAST in `plugins[]`.
That is required so static and obstacle costs are already present when
InflationLayer expands them.
