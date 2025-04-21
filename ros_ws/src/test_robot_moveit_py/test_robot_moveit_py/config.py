import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

moveit_config = (
        MoveItConfigsBuilder(robot_name="six_axis_robot", 
            package_name="test_robot_moveit_config")
        .robot_description(file_path='/root/ros_ws/src/test_robot_moveit_config/config/six_axis_robot.urdf.xacro')
        .trajectory_execution(file_path=os.path.join(
                        get_package_share_directory("test_robot_moveit_config"),
                        "config",
                        "moveit_controllers.yaml"
                ))
        .moveit_cpp(
                file_path=os.path.join(
                        get_package_share_directory("test_robot_moveit_py"),
                        "config",
                        "motion_planning_python_api_tutorial.yaml"
                )
                
        )
        .to_moveit_configs()
).to_dict()
