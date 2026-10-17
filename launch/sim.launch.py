import csv
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

PACKAGE = 'dist_project'
# Height of the UWB anchors on the ceiling
TAG_HEIGHT = 3.0
# Seconds to wait after spawning before the robot nodes are started
INITIALIZATION_TIME = 5.0


def read_coordinates(path):
    with open(path) as f:
        return [[float(x) for x in row] for row in csv.reader(f) if row]


def spawn(name, x=0.0, y=0.0, z=0.0, file=None, string=None):
    # Entities are added to the running Gazebo world through the ros_gz_sim create node
    args = ['-world', 'room', '-name', name, '-x', str(x), '-y', str(y), '-z', str(z)]
    args += ['-file', file] if file else ['-string', string]
    return Node(package='ros_gz_sim', executable='create', arguments=args, output='log')


def spawn_robots(context, xacro_file, robot_names, robots):
    # The robot description depends on the camera_rate argument, known only when the launch is running
    camera_rate = context.launch_configurations['camera_rate']
    return [spawn(name, x, y, 0.1, string=xacro.process_file(
                xacro_file, mappings={'robot_name': name, 'camera_rate': camera_rate}).toxml())
            for name, (x, y) in zip(robot_names, robots)]


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    params = os.path.join(share, 'config', 'params.yaml')
    coordinates = {name: os.path.join(share, 'coordinates', name + '_coordinates.csv')
                   for name in ('tag', 'target', 'robot')}

    # Scene read from the csv files: one robot or anchor per line
    tags = read_coordinates(coordinates['tag'])
    targets = read_coordinates(coordinates['target'])
    robots = read_coordinates(coordinates['robot'])
    robot_names = ['robot' + str(i) for i in range(len(robots))]

    gui = LaunchConfiguration('gui')
    monitor = LaunchConfiguration('monitor')
    record = LaunchConfiguration('record')
    overview_camera = LaunchConfiguration('overview_camera')

    arguments = [
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI with the robot cameras and live monitor'),
        DeclareLaunchArgument('monitor', default_value='true', description='Live view of map, estimates and errors'),
        DeclareLaunchArgument('record', default_value='false', description='Record a bag of the estimates'),
        DeclareLaunchArgument('bag_name', default_value='run'),
        DeclareLaunchArgument('bag_dir', default_value='bagfiles'),
        DeclareLaunchArgument('overview_camera', default_value='false',
                              description='Fixed camera above the room on /overview_camera/image_raw'),
        DeclareLaunchArgument('camera_rate', default_value='10',
                              description='Frame rate of the robot cameras, lower it (e.g. 5) without a GPU'),
    ]

    # Gazebo finds the meshes of the package (package://dist_project/...) in the share directory
    resources = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(share) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression(["'-r ' + ('' if '", gui, "' == 'true' else '-s ') + '",
                                         os.path.join(share, 'worlds', 'room.world'), "'"]),
            'on_exit_shutdown': 'true',
        }.items())

    # Room, anchors, targets and robots
    entities = [spawn('room', file=os.path.join(share, 'models', 'room', 'model.sdf'))]
    entities += [spawn('tag_' + str(i), x, y, TAG_HEIGHT, file=os.path.join(share, 'urdf', 'tag.urdf'))
                 for i, (x, y) in enumerate(tags)]
    # The target is static, so it is spawned already standing on the floor
    entities += [spawn('target_' + str(i), x, y, 0.4, file=os.path.join(share, 'models', 'target', 'model.sdf'))
                 for i, (x, y) in enumerate(targets)]
    entities.append(OpaqueFunction(function=spawn_robots, args=[
        os.path.join(share, 'urdf', 'unicycle.urdf.xacro'), robot_names, robots]))
    entities.append(Node(package='ros_gz_sim', executable='create', output='log', condition=IfCondition(overview_camera),
                         arguments=['-world', 'room', '-name', 'overview_camera',
                                    '-file', os.path.join(share, 'models', 'overview_camera', 'model.sdf')]))

    # Static transforms of the anchors, read by the robots to know where they are
    anchors = [Node(package='tf2_ros', executable='static_transform_publisher', output='log',
                    arguments=['--x', str(x), '--y', str(y), '--z', str(TAG_HEIGHT),
                               '--frame-id', 'world', '--child-frame-id', 'uwb_anchor_' + str(i)])
               for i, (x, y) in enumerate(tags)]

    # Bridge between Gazebo and ROS topics: commands in, odometry, ground truth and camera out
    topics = []
    for name in robot_names:
        topics += ['/{}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'.format(name),
                   '/{}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'.format(name),
                   '/{}/ground_truth/state@nav_msgs/msg/Odometry[gz.msgs.Odometry'.format(name),
                   '/{}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'.format(name)]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=topics, output='log')
    # Gazebo publishes the clock at every physics step (1 kHz): I give the nodes a 100 Hz clock,
    # otherwise every node spends its time reading the clock and the simulation slows down
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='log',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], remappings=[('/clock', '/gz_clock')])
    clock_throttle = Node(package='topic_tools', executable='throttle', name='clock_throttle', output='log',
                          arguments=['messages', '/gz_clock', '100', '/clock'])
    # Images with the detected target, shown in the panels of the Gazebo window
    blob_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='blob_bridge', output='log',
                       condition=IfCondition(gui),
                       arguments=['/{}/target/image_blob@sensor_msgs/msg/Image]gz.msgs.Image'.format(name)
                                  for name in robot_names])
    overview_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='log',
                           condition=IfCondition(overview_camera),
                           arguments=['/overview_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'])

    # Nodes of each robot in its own namespace
    robot_nodes = []
    for name in robot_names:
        for executable in ('uwb_dist_sim.py', 'motion_planner.py', 'kalman_localization.py',
                           'image_processing.py', 'target_estimator.py'):
            robot_nodes.append(Node(package=PACKAGE, executable=executable, namespace=name,
                                    parameters=[params], output='log'))

    viewers = [
        Node(package=PACKAGE, executable='live_monitor.py', output='log',
             parameters=[params, {'tag_coordinate_file': coordinates['tag'],
                                  'target_coordinate_file': coordinates['target'],
                                  'robot_coordinate_file': coordinates['robot']}],
             condition=IfCondition(PythonExpression(["'", gui, "' == 'true' and '", monitor, "' == 'true'"]))),
    ]

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast', '-o',
             PathJoinSubstitution([LaunchConfiguration('bag_dir'), LaunchConfiguration('bag_name')]),
             '--regex', '.*/(localization_data_topic|ground_truth/state|target_estimate)'],
        condition=IfCondition(record), output='log')

    return LaunchDescription(arguments + [
        resources, gazebo, clock_bridge, clock_throttle, bridge, blob_bridge, overview_bridge, *entities, *anchors, recorder,
        TimerAction(period=INITIALIZATION_TIME, actions=robot_nodes + viewers),
    ])
