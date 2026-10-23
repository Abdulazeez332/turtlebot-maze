# AI407 Maze Robot: Object Retrieval Simulation

A ROS 2 and Gazebo Classic project demonstrating a TurtleBot3-style robot retrieving two objects in a 7 × 7 maze, delivering them to a target cell, and returning to its starting point.

## Features

- Cell-based maze representation and breadth-first search (BFS) path planning.
- Task memory for the object location, delivery route, and return route.
- Two sequential object pickup and delivery cycles.
- Gazebo world with robot, objects, and maze geometry.
- Project report and presentation included.

## How it works

The demonstration starts at cell (1,3), follows a scripted search route to objects at (5,4), delivers each object to (1,7), and returns to the start. Robot and object poses are updated using Gazebo's `/set_entity_state` service. This is a deterministic task demonstration; physical navigation, perception, and manipulator control are not implemented.

## Project files

```text
src/ai407_maze_project/
  ai407_maze_project/maze_task_node.py
  launch/simulation.launch.py
  maps/maze_graph.json
  worlds/phase3_7x7_maze.world
  package.xml
  setup.py
report/                  # Word and PDF project report
presentation/            # PowerPoint presentation
```

## Setup and run

Use a Linux environment with ROS 2, Gazebo Classic, `gazebo_ros`, `gazebo_msgs`, and the colcon build tools installed. Source your ROS 2 installation first.

```bash
git clone https://github.com/Abdulazeez332/ai407-maze-robot.git
cd ai407-maze-robot
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch ai407_maze_project simulation.launch.py
```

The simulation requires Gazebo's `/set_entity_state` service. When the service is unavailable, the node logs intended movement instead. The provided launch file and simulation have not been validated in this publishing environment.

## Documentation

- [Project report (PDF)](report/AI407_Final_Project_Report.pdf)
- [Project report (Word)](report/AI407_Final_Project_Report.docx)
- [Presentation](presentation/AI407_Project_Presentation.pptx)

Course project: **AI407**. Original team information is retained in the supplied documents and source files.
