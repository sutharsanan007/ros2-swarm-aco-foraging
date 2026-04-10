import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim    = get_package_share_directory('ros_gz_sim')
    custom_models_dir = os.path.expanduser(
        '~/mrs_foraging_ws/src/multi_robot_foraging/models'
    )
    world_file = os.path.expanduser(
        '~/mrs_foraging_ws/src/multi_robot_foraging/worlds/foraging_arena.world'
    )

    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=custom_models_dir
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # ── Static TF: world → map ───────────────────────────────────────────
    # Without this, RViz drops every frame-stamped message with
    # "discarding message because the queue is full".
    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )

    # ── Model paths ──────────────────────────────────────────────────────
    r1_model = os.path.join(custom_models_dir, 'robot1_burger', 'model.sdf')
    r2_model = os.path.join(custom_models_dir, 'robot2_burger', 'model.sdf')
    r3_model = os.path.join(custom_models_dir, 'robot3_burger', 'model.sdf')

    # ── Spawn nodes ───────────────────────────────────────────────────────
    r1_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', 'foraging_arena', '-name', 'robot1',
            '-file', r1_model,
            '-x', '-1.5', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'
        ],
        output='screen'
    )
    r2_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', 'foraging_arena', '-name', 'robot2',
            '-file', r2_model,
            '-x', '1.5', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'
        ],
        output='screen'
    )
    r3_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', 'foraging_arena', '-name', 'robot3',
            '-file', r3_model,
            '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'
        ],
        output='screen'
    )

    # ── Agent nodes ───────────────────────────────────────────────────────
    # spawn_x / spawn_y tell the agent where it starts in world frame so it
    # can convert odom-relative navigation targets back to world coordinates
    # for pheromone drops, and vice-versa.
    r1_agent = Node(
        package='multi_robot_foraging', executable='unified_agent',
        namespace='robot1', name='unified_agent', output='screen',
        parameters=[{'resource_id': 1, 'spawn_x': -1.5, 'spawn_y': 0.0}]
    )
    r2_agent = Node(
        package='multi_robot_foraging', executable='unified_agent',
        namespace='robot2', name='unified_agent', output='screen',
        parameters=[{'resource_id': 2, 'spawn_x':  1.5, 'spawn_y': 0.0}]
    )
    r3_agent = Node(
        package='multi_robot_foraging', executable='unified_agent',
        namespace='robot3', name='unified_agent', output='screen',
        parameters=[{'resource_id': 3, 'spawn_x':  0.0, 'spawn_y': 0.0}]
    )

    # ── GZ ↔ ROS bridges ─────────────────────────────────────────────────
    # Single shared clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    # Per-robot: cmd_vel (ROS→GZ) and odom (GZ→ROS)
    r1_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='r1_bridge',
        arguments=[
            '/robot1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/robot1/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )
    r2_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='r2_bridge',
        arguments=[
            '/robot2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/robot2/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )
    r3_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='r3_bridge',
        arguments=[
            '/robot3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/robot3/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    # ── Pheromone mapper ─────────────────────────────────────────────────
    # Publishes colored MarkerArray trails (not grayscale OccupancyGrid).
    # Robot 1 = green, Robot 2 = red, Robot 3 = blue.
    # After sim completes: Robot 1 turns gold, merged segments turn white.
    phero_map = Node(
        package='multi_robot_foraging',
        executable='pheromone_mapper',
        name='pheromone_mapper',
        output='screen'
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    # After launch, configure RViz manually once:
    #   1. Global Options → Fixed Frame = 'world'
    #   2. Add → By Topic → /pheromone_trails  (MarkerArray) live colored trails
    #   3. Add → By Topic → /final_path_marker (MarkerArray) frozen final paths
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        set_model_path,

        # TF tree first — must exist before RViz and any frame-stamped publisher
        world_tf,

        # Gazebo simulator
        gazebo,

        # Bridges before agents (topics must exist before nodes publish to them)
        clock_bridge,
        r1_bridge,
        r2_bridge,
        r3_bridge,

        # Spawn robots into Gazebo
        r1_spawn,
        r2_spawn,
        r3_spawn,

        # Agent brains
        r1_agent,
        r2_agent,
        r3_agent,

        # Pheromone visualisation
        phero_map,

        # RViz
        rviz,
    ])