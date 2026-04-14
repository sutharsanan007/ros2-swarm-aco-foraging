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

    # Static TF: world → map (required for RViz frame resolution)
    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )

    # ── Model paths ───────────────────────────────────────────────────────
    r1_model = os.path.join(custom_models_dir, 'robot1_burger', 'model.sdf')
    r2_model = os.path.join(custom_models_dir, 'robot2_burger', 'model.sdf')
    r3_model = os.path.join(custom_models_dir, 'robot3_burger', 'model.sdf')

    # ── Spawn robots ──────────────────────────────────────────────────────
    # Positions MUST match SPAWN_X in unified_agent.py
    # R1(SHORT): -1.0  R3(MEDIUM): 0.0  R2(LONG): 1.0
    r1_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'foraging_arena', '-name', 'robot1',
                   '-file', r1_model,
                   '-x', '-1.0', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'],
        output='screen'
    )
    r2_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'foraging_arena', '-name', 'robot2',
                   '-file', r2_model,
                   '-x', '1.0', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'],
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
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    r1_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='r1_bridge',
        arguments=['/robot1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen'
    )
    r2_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='r2_bridge',
        arguments=['/robot2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen'
    )
    r3_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='r3_bridge',
        arguments=['/robot3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen'
    )

    # ── Pheromone mapper (RViz heatmap) ───────────────────────────────────
    phero_map = Node(
        package='multi_robot_foraging',
        executable='pheromone_mapper',
        name='pheromone_mapper',
        output='screen'
    )

    # ── Gazebo trail node (colored dots in Gazebo window) ─────────────────
    # Subscribes to /robotN/trail_pos — spawns colored cylinders as robots move
    # GREEN=Robot1, RED=Robot2, BLUE=Robot3
    # Dots grow larger each iteration to show pheromone reinforcement
    gazebo_trail = Node(
        package='multi_robot_foraging',
        executable='gazebo_trail_node',
        name='gazebo_trail_node',
        output='screen'
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2', output='screen'
    )

    return LaunchDescription([
        set_model_path,
        world_tf,
        gazebo,
        clock_bridge,
        r1_bridge, r2_bridge, r3_bridge,
        r1_spawn, r2_spawn, r3_spawn,
        r1_agent, r2_agent, r3_agent,
        phero_map,
        gazebo_trail,
        rviz,
    ])