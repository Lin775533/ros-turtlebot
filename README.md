# ROS Turtlesim Navigation Lab

**Author: Yu-Chun Lin**

## Overview

This repository contains the ROS lab assignment demonstrating basic turtle navigation using the turtlesim package. Two main Python nodes were implemented:

1. **swim_node.py**: Controls the turtle to swim in continuous figure-eight patterns by alternating between left and right circular movements.

2. **swim_to_goal.py**: Implements a proportional controller that allows the turtle to navigate to user-specified (x,y) coordinates.

## Repository Structure

```
Lab3_YuChun_Lin_ws/
├── build/                  # Build directory
├── devel/                  # Development space
├── src/                    # Source code
│   ├── autoturtle/         # ROS package
│   │   ├── scripts/
│   │   │   ├── swim_node.py
│   │   │   └── swim_to_goal.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
└── catkin_workspace       # Workspace config file
```

## Implementation Details

### swim_node.py

This node makes the turtle swim in a continuous pattern of alternating circles. Key features:
- Randomly selects a linear velocity at startup
- Calculates appropriate angular velocity based on a fixed radius
- Alternates between positive and negative angular velocities to create left/right turns
- Maintains a constant publishing rate for smooth motion

### swim_to_goal.py

This node implements a proportional controller that guides the turtle to a user-specified goal. Key features:
- Subscribes to the turtle's pose to get current position and orientation
- Calculates Euclidean distance and angle to goal
- Uses proportional control for both linear and angular velocities
- Accepts user input for goal coordinates
- Stops when the turtle reaches within tolerance of the goal

## Running the Code

1. Start ROS master:
   ```
   roscore
   ```

2. Launch turtlesim in a new terminal:
   ```
   rosrun turtlesim turtlesim_node
   ```

3. Run either of the navigation nodes:
   ```
   # For figure-eight swimming pattern
   rosrun autoturtle swim_node.py
   
   # For goal-directed navigation
   rosrun autoturtle swim_to_goal.py
   ```

## Demo Images

![Turtlesim Navigation Demo](screenshots/turtlesim_demo.png)

## Technologies Used

- ROS (Robot Operating System)
- Python
- turtlesim package
- Publisher/Subscriber pattern
- Proportional control for navigation

## Learning Outcomes

- Understanding of ROS node creation and communication
- Implementation of basic robot movement algorithms
- Working with publishers and subscribers
- Development of a simple proportional controller
- Integration of user input with robot control
