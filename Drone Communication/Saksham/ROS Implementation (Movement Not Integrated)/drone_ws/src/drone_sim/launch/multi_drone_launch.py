from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    THIS_DIR = os.path.dirname(os.path.realpath(__file__))
    SDF_DIR = os.path.expanduser(os.path.join(
        os.path.dirname(os.path.dirname(THIS_DIR)),
        'drone_sim', 'sdf'
    ))

    drone_a_sdf = os.path.join(SDF_DIR, 'drone_a.sdf')
    drone_b_sdf = os.path.join(SDF_DIR, 'drone_b.sdf')

    # 1) Start Gazebo in main terminal
    start_gz = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', os.path.join(SDF_DIR, 'empty_world.sdf')],
        output='screen'
    )

    # 2) Spawn drones
    spawn_a = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-entity', 'drone_a', '-file', drone_a_sdf,
             '-x', '0', '-y', '0', '-z', '1'],
        output='screen'
    )

    spawn_b = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-entity', 'drone_b', '-file', drone_b_sdf,
             '-x', '2', '-y', '0', '-z', '1'],
        output='screen'
    )

    # 3) File-based comm & move nodes
    start_comm_a = Node(
        package='drone_sim',
        executable='comm_node',
        arguments=['drone_a', os.path.join(SDF_DIR, '../test_inputs/drone_a_comm.txt'), '/drone_A_to_B'],
        output='screen'
    )

    start_recv_b = Node(
        package='drone_sim',
        executable='receive_node',
        arguments=['drone_b', '/drone_A_to_B'],
        output='screen'
    )

    start_comm_b = Node(
        package='drone_sim',
        executable='comm_node',
        arguments=['drone_b', os.path.join(SDF_DIR, '../test_inputs/drone_b_comm.txt'), '/drone_B_to_A'],
        output='screen'
    )

    start_recv_a = Node(
        package='drone_sim',
        executable='receive_node',
        arguments=['drone_a', '/drone_B_to_A'],
        output='screen'
    )

    start_move_a = Node(
        package='drone_sim',
        executable='move_node',
        arguments=['drone_a', os.path.join(SDF_DIR, '../test_inputs/drone_a_move.txt')],
        output='screen'
    )

    start_move_b = Node(
        package='drone_sim',
        executable='move_node',
        arguments=['drone_b', os.path.join(SDF_DIR, '../test_inputs/drone_b_move.txt')],
        output='screen'
    )

    # 4) Realtime communication nodes in separate terminals
    drone_a_realtime_term = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'bash', '-c',
            'ros2 run drone_sim realtime_comm_node drone_a /drone_A_to_B; exec bash'
        ],
        output='screen'
    )

    drone_b_realtime_term = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'bash', '-c',
            'ros2 run drone_sim realtime_comm_node drone_b /drone_B_to_A; exec bash'
        ],
        output='screen'
    )

    recv_a_term = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'bash', '-c',
            'ros2 run drone_sim receive_node drone_a /drone_B_to_A; exec bash'
        ],
        output='screen'
    )

    recv_b_term = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'bash', '-c',
            'ros2 run drone_sim receive_node drone_b /drone_A_to_B; exec bash'
        ],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        start_gz,
        TimerAction(period=3.0, actions=[spawn_a]),
        TimerAction(period=6.0, actions=[spawn_b]),
        TimerAction(period=7.0, actions=[
            start_comm_a,
            start_comm_b,
            start_move_a, start_move_b,

            drone_a_realtime_term, drone_b_realtime_term,
            recv_a_term, recv_b_term
        ]),
    ])
