# Scripts

These python scripts hold all the nodes and logic of the project

## `init_gazebo.py`
## Key Features
- **Spawns Models in Gazebo**: Dynamically spawns tags, targets, robots, and the room model in the Gazebo simulation environment using URDF and SDF files.
- **Publishes Static Transforms**: Continuously publishes static transforms for the tags, aiding in localization tasks.
- **Initializes Nodes**: Recursively launches additional ROS nodes for each robot based on the specified number of robots in a CSV file.

## Topics
The script interacts with the following topics:
- `/gazebo/spawn_urdf_model`: Service used to spawn URDF models in Gazebo.
- `/gazebo/spawn_sdf_model`: Service used to spawn SDF models in Gazebo.
- Static transforms are published for tags to the `/tf_static` topic using `tf2_ros.StaticTransformBroadcaster`.

## Parameters
The script imports various parameters from the `deploy.launch` file:
- **Tag Parameters**:
  - `tag_height`: Height at which to spawn tags.
  - `tag_distance_rate`: Frequency at which to publish tag positions.
  - `tag_coordinate_file`: CSV file containing coordinates for tags.
  - `tag_urdf_file`: URDF file for tag models.
- **Target Parameters**:
  - `target_coordinate_file`: CSV file containing coordinates for targets.
  - `target_sdf_file`: SDF file for target models.
- **Room Parameters**:
  - `room_sdf_file`: SDF file for the room model.
- **Robot Parameters**:
  - `robot_coordinate_file`: CSV file containing initial coordinates for robots.
  - `robot_description`: URDF description of the robot.
- **General Parameters**:
  - `initialization_time`: Time to wait before starting the initialization process.

## Functions
- **`spawn_tag(x, y, id)`**: Spawns a tag at the given coordinates using a URDF model.
- **`spawn_target(x, y, id)`**: Spawns a target at the given coordinates using an SDF model.
- **`spawn_robot(x, y, id)`**: Spawns a robot at the given coordinates using a URDF model.
- **`spawn_room()`**: Spawns the room model in the simulation using an SDF file.
- **`publish_static_transform(x, y, id)`**: Publishes the static transform for a tag.

## Usage
To use the script, ensure the ROS environment is properly set up and the necessary parameters are defined in the `deploy.launch` file. Run the script to initialize the Gazebo environment, spawn the models, and start the required nodes.

```bash
rosrun your_package init_gazebo.py
