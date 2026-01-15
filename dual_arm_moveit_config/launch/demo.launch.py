import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError as e:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError as e:
        return None

def generate_launch_description():
    # 1. 基础资源加载
    urdf_file = os.path.join(get_package_share_directory('dual_arm_description'), 'urdf', 'dual_panda.urdf.xacro')
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    robot_description_semantic = {'robot_description_semantic': load_file('dual_arm_moveit_config', 'config/dual_panda.srdf')}
    kinematics_yaml = load_yaml('dual_arm_moveit_config', 'config/kinematics.yaml')
    moveit_controllers = load_yaml('dual_arm_moveit_config', 'config/controllers.yaml')
    
    ompl_planning_pipeline_config = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateCollision""",
        'start_state_max_bounds_error': 0.1,
    }}
    ompl_yaml = load_yaml('dual_arm_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    # 2. Move Group Node
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers,
            # 使用 SimpleController 防止报错
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'moveit_manage_controllers': True},
            {'use_sim_time': False},
            {'publish_robot_description_semantic': True},
            # 关闭执行监控，防止 -4 错误
            {'trajectory_execution.allowed_execution_duration_scaling': 5.0}, 
            {'trajectory_execution.execution_duration_monitoring': False},
        ],
    )

    # 3. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    # 4. Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # 5. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 注意：我们移除了 mock_hardware.py 的自动运行，将在外部手动运行它！

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        rviz_node
    ])



