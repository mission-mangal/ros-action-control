# ROS ActionLib Example

This repository contains an example demonstrating the usage of ROS (Robot Operating System) ActionLib for communication between nodes. The example includes both an action server and client setup, showcasing how to send and receive goals, handle feedback, and process results. The action, named `MissionAction`, deals with various data types including float, arrays, and boolean.

## Getting Started

### Prerequisites

- [ROS](http://www.ros.org/) installed on your system
- Catkin workspace set up

### Installation

1. Clone the repository to your catkin workspace:

    ```bash
    git clone https://github.com/your-username/ros-actions.git
    ```

2. Build the workspace:

    ```bash
    cd path/to/catkin/workspace
    catkin_make
    ```

3. Source your workspace:

    ```bash
    source devel/setup.bash
    ```

### Usage

1. Launch the ROS core:

    ```bash
    roscore
    ```

2. Open a new terminal and run the action server:

    ```bash
    roslaunch action_setup mission_server.launch
    ```

3. Open another terminal and run the action client:

    ```bash
    roslaunch action_setup mission_client.launch
    ```

### Action Definition

The `MissionAction` action is defined as follows:

```bash
# MissionAction.action
float32 yaw_angle
int64[] x_coordinates
int64[] y_coordinates
bool obstacle_info
---
string result
---
string feedback
