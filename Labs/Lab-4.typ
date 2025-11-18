#import "Class.typ": *

#show: ieee.with(
  title: [#text(smallcaps("Lab #4: SLAM and Nav2 (ROS 2 Humble)"))],
  /*
  abstract: [
    #lorem(10).
  ],
  */
  authors:
  (
    (
      name: "Abdelbacet Mhamdi",
      department: [Senior-lecturer, Dept. of EE],
      organization: [ISET Bizerte --- Tunisia],
      profile: "a-mhamdi",
    ),

    /*
    (
      name: "Student 1",
      department: [Dept. of EE],
      organization: [ISET Bizerte --- Tunisia],
      profile: "abc",
    ),
    (
      name: "Student 2",
      department: [Dept. of EE],
      organization: [ISET Bizerte --- Tunisia],
      profile: "abc",
    ),
    (
      name: "Student 3",
      department: [Dept. of EE],
      organization: [ISET Bizerte --- Tunisia],
      profile: "abc",
    )
  */

  )
  // index-terms: (""),
  // bibliography-file: "Biblio.bib",
)

= SLAM and Nav2 with `my_robot`

This lab guides you through simulation, mapping (SLAM Toolbox), and localization/navigation (Nav2) using `my_robot`.

The following prerequisites are required:

- *ROS 2* Humble installed and sourced
- Gazebo and RViz2 installed
- `my_robot` built and sourced
```bash
colcon build && source install/setup.bash
```
- Robot publishes `/robot_description` and a laser `/scan`

== Simulation + RViz

Displays the robot in Gazebo and publishes `/robot_description` and `/scan` for RViz.

```bash
ros2 launch my_robot launch_sim.launch.py
```

In RViz:

- Set Fixed Frame (e.g., `odom`)
- Add `RobotModel` (uses `/robot_description`)
- Add `LaserScan` on `/scan`

== Mapping with SLAM Toolbox

Start online SLAM and drive the robot to cover the space.

```bash
ros2 launch my_robot launch_slam.launch.py
```

#tip[
  - Teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
  - Check topics/frames: `ros2 topic list`, `ros2 run tf2_tools view_frames`
  - View map in RViz: add `Map` (topic `/map`)
]

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_world
```

== Localization and Navigation (Nav2)

Launch Nav2 (AMCL + planners + controllers) and navigate in RViz.

```bash
ros2 launch my_robot launch_nav.launch.py
```

The overall workflow is as follows:

- In RViz set `Fixed Frame` to `map`
- Use `2D Pose Estimate` to set initial pose
- Use `Nav Goal` to send a goal

/*
== Troubleshooting

- No laser in RViz: ensure `/scan` exists and frame IDs match TF tree
- Nav2 idle or oscillating: verify transforms `map->odom->base_link`, costmap params, and footprint
- Map not saving: confirm `/map` is being published by SLAM
*/

== Summary

```bash
# Simulation
ros2 launch my_robot launch_sim.launch.py

# Mapping
ros2 launch my_robot launch_slam.launch.py

# Localization + Navigation
ros2 launch my_robot launch_nav.launch.py
```
