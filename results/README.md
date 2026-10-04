# Simulation results

Run the MATLAB scripts and export a small set of representative figures to this directory before publishing the repository.

Recommended filenames:

- `altitude_pid_response.png`
- `point_tracking_3d.png`
- `smooth_waypoint_tracking.png`
- `lqr_hover_response.png`
- `kalman_position_estimation.png`

In a MATLAB figure window, use **File > Save As**, or run:

```matlab
exportgraphics(gcf, 'results/figure_name.png', 'Resolution', 200)
```

Do not add every generated plot. Four or five clear figures are more useful than dozens of nearly identical screenshots.

