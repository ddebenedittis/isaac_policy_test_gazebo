import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from scripts import GazeboRosPaths



def generate_launch_description():
    
    global robot_name, robot_file_path, gazebo_config_file_path
    global terrain_file_path, terrain_increasing_slope_file_path, heightmap_terrain_file_path
    global world_file_path
    
    robot_name = LaunchConfiguration('robot_name', default='anymal_c')

    robot_file_path = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration('package_name', default="anymal_c_simple_description")),
        LaunchConfiguration('robot_file_path', default=os.path.join('urdf', 'anymal.xacro'))
    ])
    
    gazebo_config_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'config/gazebo_params.yaml'
    ])
    
    terrain_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'objects/terrain.xacro'
    ])
    
    terrain_increasing_slope_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'objects/terrain_increasing_slope.xacro'
    ])
    
    heightmap_terrain_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'objects/heightmap_terrain.xacro'
    ])

    world_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        LaunchConfiguration('world_file_path', default=os.path.join('worlds', 'anymal.world'))
    ])
    
    
    global velocity_cmd, height, terrain, use_sim_time
    
    velocity_cmd = LaunchConfiguration('velocity_cmd', default='[0., 0., 0.]')

    height = LaunchConfiguration('height', default='0.64')
    
    terrain = LaunchConfiguration('terrain', default='rigid')
        
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    
    # ======================================================================== #

    ld =  LaunchDescription([
        
        DeclareLaunchArgument('contact_constraint_type', default_value="'soft_kv'"),
                
        DeclareLaunchArgument('velocity_cmd', default_value='[0., 0., 0.]'),
        SetParameter(name='velocity_cmd', value=velocity_cmd),

        DeclareLaunchArgument('terrain', default_value='rigid'),
    ])
    
    launch_generate_dae_files(ld)
        
    launch_gazebo(ld)
        
    spawn_things(ld)
    
    spawn_controllers(ld)
        
    return ld


# ========================= Launch_generate_dae_files ======================== #

def launch_generate_dae_files(ld):
    
    global heightmap_mesh_generator
    
    heightmap_mesh_generator = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '"', ' == "heightmap"'
            ])
        ),
        package = 'robot_gazebo',
        executable = 'generate_heightmap_mesh.py',
        output = 'screen',
    )
    
    ld.add_action(heightmap_mesh_generator)


# =============================== Launch_gazebo ============================== #

def launch_gazebo(ld):
    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']
    
    gazebo_server = ExecuteProcess(
        cmd=[['ros2 launch gazebo_ros gzserver.launch.py verbose:=true pause:=true world:=', world_file_path, ' params_file:=', gazebo_config_file_path]],
        additional_env={'__NV_PRIME_RENDER_OFFLOAD': '1',
                        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
                        'GAZEBO_MODEL_PATH': model,
                        'GAZEBO_PLUGIN_PATH': plugin,
                        'GAZEBO_RESOURCE_PATH': media},
        shell=True,
        output = 'screen',
    )
    
    gazebo_client = ExecuteProcess(
        cmd=[['ros2 launch gazebo_ros gzclient.launch.py']],
        additional_env={'__NV_PRIME_RENDER_OFFLOAD': '1',
                        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'},
        shell=True,
        output = 'screen',
    )
    
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    

# =============================== Spawn_things =============================== #

def spawn_things(ld):
    """Spawn the robot and the terrain."""
    
    # Spawn the robot
    
    robot_rsp = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', robot_file_path, ' remove_shank_collisions:=True']), value_type=str)},
        ],
        output = 'screen',
    )

    spawn_robot = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-topic', '/robot_description',
                     '-entity', robot_name,
                     '-x', '0', '-y', '0', '-z', height,
                     '-R', '0', '-P', '0', '-Y', '0',],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    
    # Spawn the terrain
    
    terrain_rsp = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '" == "rigid" or "', terrain, '" == "soft" or ',
                '"', terrain, '" == "very_soft"'
                
            ])
        ),
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'ground_plane',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', terrain_file_path, ' ', 'terrain:=', terrain]), value_type=str)}
        ],
        output = 'screen',
    )
    
    terrain_increasing_slope_rsp = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '"', ' == "increasing_slope"'
            ])
        ),
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'ground_plane',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', terrain_increasing_slope_file_path, ' ', 'terrain:=', terrain]), value_type=str)}
        ],
        output = 'screen',
    )
    
    heightmap_terrain_rsp = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '"', ' == "heightmap"'
            ])
        ),
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'ground_plane',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', heightmap_terrain_file_path, ' ', 'terrain:=', terrain]), value_type=str)}
        ],
        output = 'screen',
    )
    
    heightmap_terrain_rsp_ev_hdl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=heightmap_mesh_generator,
            on_exit=[heightmap_terrain_rsp],
        )
    )
        
    spawn_terrain = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-entity', 'ground_plane',
                     '-topic', '/ground_plane/robot_description',
                     '-x', '0', '-y', '0', '-z', '0',
                     '-R', '0', '-P', '0', '-Y', '0'],
        output = 'screen',
    )
    
    ld.add_action(robot_rsp)
    ld.add_action(spawn_robot)
    
    ld.add_action(terrain_rsp)
    ld.add_action(terrain_increasing_slope_rsp)
    ld.add_action(heightmap_terrain_rsp_ev_hdl)
    ld.add_action(spawn_terrain)
    

# ============================= Spawn_controllers ============================ #

def spawn_controllers(ld):
    
    spawn_joint_state_broadcaster = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    spawn_imu_sensor_broadcaster = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    
    spawn_pd_control = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['pd_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time},],
        emulate_tty=True,
        output='screen',
    )
    
    spawn_policy_inference = Node(
        package='policy_inference',
        executable='policy_inference_node',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True,
        output='screen',
    )

    spawn_teleop_trot = Node(
        package='teleoperate_robot',
        executable='teleop_velocity_command_node',
        name='teleoperate_robot',
        prefix=['xterm -fg white -bg black -e'],
        output='screen',
        parameters=[{'robot_name': robot_name}],
    )
    
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(spawn_imu_sensor_broadcaster)
    ld.add_action(spawn_pd_control)
    ld.add_action(spawn_policy_inference)
    ld.add_action(spawn_teleop_trot)
