#! /usr/bin/env python3
import typing
import rclpy
from rclpy.node import Node

# message libraries
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryMsg
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from moveit_msgs.msg import DisplayTrajectory as DisplayTrajectoryMsg

from moveit import (MoveItPy,
    PlanRequestParameters)
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory
from .config import moveit_config


class ExecutorNode(Node):
    def __init__(self, robot_interface):
        super().__init__("executor_node")
        self._planning_group_name = "arm"
        self._logger = rclpy.logging.get_logger("moveit_py.Robot")
        self._robot_interface = robot_interface
        self._plan_request_parameters = PlanRequestParameters(self._robot_interface, 
            "ompl_planning_parameters")
        self._display_trajectory_publisher = self.create_publisher(DisplayTrajectoryMsg, '/display_planned_path', 10)

    def plan(self, poses: typing.List[PoseStamped],
             display_planned_path: typing.Optional[bool] = False,
             execute_trajectory: typing.Optional[bool] = False) -> RobotTrajectory:
        self._logger.info("planning trajectory")
        robot_planning = self._robot_interface.get_planning_component(self._planning_group_name)
        robot_planning.set_start_state_to_current_state() # resets start state in planning component
        plans = []
        for pose in poses:
            robot_planning.set_goal_state(pose_stamped_msg=pose, pose_link="r1_end_effector")
            plan_result = robot_planning.plan(single_plan_parameters=self._plan_request_parameters)
            if not plan_result: raise RuntimeError("Planning failed")
            
            trajectory = plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory
            plans.append(trajectory)
            
            # now set the start state for the next plan to the end state of the current plan
            robot_state = RobotState(self._robot_interface.get_robot_model())

            last_joint_positions = {joint: pos for joint, pos in \
                zip(trajectory.joint_names, trajectory.points[-1].positions)}
            
            robot_state.set_joint_group_active_positions(self._planning_group_name, \
                [last_joint_positions[joint] for joint in trajectory.joint_names])

            robot_planning.set_start_state(robot_state=robot_state)
            
        #  now combine all plans into one
        robot_trajectory = RobotTrajectory(self._robot_interface.get_robot_model())
        
        first_joint_positions = {joint: pos for joint, pos in \
            zip(plans[0].joint_names, plans[0].points[0].positions)}
        first_robot_state = RobotState(self._robot_interface.get_robot_model())
        first_robot_state.set_joint_group_active_positions(self._planning_group_name, \
            [first_joint_positions[joint] for joint in plans[0].joint_names])
        
        joint_trajectory = JointTrajectoryMsg()
        joint_trajectory.joint_names = plans[0].joint_names
        joint_trajectory.points = []
        for plan in plans:
            joint_trajectory.points.extend(plan.points)
        robot_trajectory_msg = RobotTrajectoryMsg()
        robot_trajectory_msg.joint_trajectory = joint_trajectory

        if display_planned_path:
            self._display_trajectory(robot_trajectory_msg)

        robot_trajectory.set_robot_trajectory_msg(first_robot_state, robot_trajectory_msg)
        robot_trajectory.joint_model_group_name = self._planning_group_name
        robot_trajectory.apply_totg_time_parameterization(1.0, 1.0)

        if execute_trajectory:
            self._robot_interface.execute(robot_trajectory, controllers=[])

    def _display_trajectory(self, trajectory_msg: RobotTrajectory):
        display_trajectory = DisplayTrajectoryMsg()
        display_trajectory.model_id = "six_axis_robot"
        display_trajectory.trajectory.append(trajectory_msg)
        self._display_trajectory_publisher.publish(display_trajectory)


def main(args=None):
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    robot_if = MoveItPy(node_name="moveit_py", config_dict=moveit_config,
        launch_params_filepaths=["params.yaml"])

    logger.info("MoveItPy instance created")

    executor = ExecutorNode(robot_if)
    target_pose = Pose()
    target_pose.position.x = -0.3
    target_pose.position.y = 0.2
    target_pose.position.z = 1.0
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0

    target_pose_stamped = PoseStamped()
    target_pose_stamped.header.frame_id = "base_link"
    target_pose_stamped.pose = target_pose
    _plan = executor.plan([target_pose_stamped],
                        display_planned_path=True,
                        execute_trajectory=False)
    logger.info("Plan created")
    executor.destroy_node()
    robot_if.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
