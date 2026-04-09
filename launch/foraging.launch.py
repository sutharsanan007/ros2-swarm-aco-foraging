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

    # ── THE FIX: Static TF publisher ────────────────────────────────────
    # RViz needs a TF tree to display any frame-stamped message.
    # Without this, every OccupancyGrid with frame_id='world' is dropped
    # with "discarding message because the queue is full".
    # This node publishes: world (parent) → map (child), identity transform.
    # RViz with Fixed Frame = 'world' will now render the pheromone grids.
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
        arguments=['-world', 'foraging_arena', '-name', 'robot1',
                   '-file', r1_model,
                   '-x', '-1.5', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'],
        output='screen'
    )
    r2_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'foraging_arena', '-name', 'robot2',
                   '-file', r2_model,
                   '-x', '1.5', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'],
        output='screen'
    )
    r3_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'foraging_arena', '-name', 'robot3',
                   '-file', r3_model,
                   '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'],
        output='screen'
    )

    # ── Agent nodes ───────────────────────────────────────────────────────
    r1_agent = Node(
        package='multi_robot_foraging', executable='unified_agent',
        namespace='robot1', name='unified_agent', output='screen',
        parameters=[{'resource_id': 1}]
    )
    r2_agent = Node(
        package='multi_robot_foraging', executable='unified_agent',
        namespace='robot2', name='unified_agent', output='screen',
        parameters=[{'resource_id': 2}]
    )
    r3_agent = Node(
        package='multi_robot_foraging', executable='unified_agent',
        namespace='robot3', name='unified_agent', output='screen',
        parameters=[{'resource_id': 3}]
    )

    # ── GZ ↔ ROS bridges ─────────────────────────────────────────────────
    # Clock bridge is shared — only needs to be launched once.
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Per-robot bridges: cmd_vel (ROS→GZ) + odom (GZ→ROS)
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

    # ── Pheromone mapper (the only swarm visualisation node needed) ───────
    phero_map = Node(
        package='multi_robot_foraging',
        executable='pheromone_mapper',
        name='pheromone_mapper',
        output='screen'
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    # We launch WITHOUT a config so you can set up displays interactively.
    # In RViz:
    #   1. Set Fixed Frame = 'world'
    #   2. Add → By Topic → /pheromone_map   (Map display, live heatmap)
    #   3. Add → By Topic → /final_path_map  (Map display, frozen winner)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        set_model_path,

        # TF tree FIRST — must exist before RViz starts
        world_tf,

        # Gazebo
        gazebo,

        # Bridges before agents (topics must exist before publishers start)
        clock_bridge,
        r1_bridge, r2_bridge, r3_bridge,

        # Spawn robots
        r1_spawn, r2_spawn, r3_spawn,

        # Agent brains
        r1_agent, r2_agent, r3_agent,

        # Swarm visualisation
        phero_map,

        # RViz
        rviz,
    ])