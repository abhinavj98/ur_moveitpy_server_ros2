import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
from sensor_msgs.msg import Joy
from moveit_py_server_interfaces.srv import MoveToPose, MoveToJoints


class MoveitPyServer(Node):
    def __init__(self):
        super().__init__('moveit_py_server')
        time.sleep(5)
        self.cb = rclpy.callback_groups.ReentrantCallbackGroup()
        self.robot = MoveItPy(node_name='moveit_py_server')
        self.get_logger().info("MoveItPy instance created")
        self.planning_component = self.robot.get_planning_component("ur_manipulator")
        
        self.srv_pose = self.create_service(MoveToPose, 'move_to_pose', self.move_to_pose_callback, callback_group=self.cb)
        self.srv_joints = self.create_service(MoveToJoints, 'move_to_joints', self.move_to_joints_callback, callback_group=self.cb)
    
    def move_to_pose_callback(self, request, response):
        self.get_logger().info(f"Received request to move to pose: {request.target_pose}")
        planning_scene_monitor = self.robot.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            robot_state = scene.current_state
            robot_state.update()
            original_joint_positions = robot_state.get_joint_group_positions("ur_manipulator")
            
            self.get_logger().info("Original joint positions: " +  str(original_joint_positions))
            self.planning_component.set_start_state_to_current_state()

            check_init_pose = robot_state.get_pose("tool0")
            self.get_logger().info("Initial_pose:" + str(check_init_pose))

            # randomize the robot state
            robot_state.set_from_ik(joint_model_group_name="ur_manipulator",
                geometry_pose=request.target_pose,
                tip_name="tool0",
                timeout=1.0)

            robot_state.update()

            check_updated_pose = robot_state.get_pose("tool0")
            updated_joint_positions = robot_state.get_joint_group_positions("ur_manipulator")
            self.planning_component.set_goal_state(robot_state=robot_state)
            robot_state.update()

            self.get_logger().info("New_pose:" + str(check_updated_pose))
            self.get_logger().info("Updated joint positions: " +  str(updated_joint_positions))

            #Set robot state to original state
            robot_state.set_joint_group_positions("ur_manipulator", original_joint_positions)
            robot_state.update()

            # set goal state to the initialized robot state
            # self.logger.info("Go to goal")
            self.get_logger().info("Go to goal")
            
         # plan to goal
        plan_result = self.plan()
            
        # execute the plan
        if plan_result:
            self.get_logger().info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
            #TODO: Check if execution was successful
            response.success = True
            response.message = "Motion executed successfully"
        else:
            self.get_logger().error("Planning failed")
            response.success = False
            response.message = "Planning failed"

        return response

    def move_to_joints_callback(self, request, response):
        self.get_logger().info(f"Received request to move to joints: {request.target_joints}")

        planning_scene_monitor = self.robot.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            robot_state = scene.current_state
            robot_state.update()
            og_joint_positions = robot_state.get_joint_group_positions("ur_manipulator")
            self.planning_component.set_start_state_to_current_state()
            robot_state.set_joint_group_positions("ur_manipulator", request.target_joints)
            robot_state.update()
            self.planning_component.set_goal_state(robot_state=robot_state)
            robot_state.set_joint_group_positions("ur_manipulator", og_joint_positions)
            robot_state.update()

        plan_result = self.plan()
        if plan_result:
            self.get_logger().info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
            response.success = True
            response.message = "Motion executed successfully"
        else:
            self.get_logger().error("Planning failed")
            response.success = False
            response.message = "Planning failed"
        
        return response

    def plan(self, single_plan_parameters=None, multi_plan_parameters=None, sleep_time=0.0):
        """Helper function to plan and execute a motion."""
        self.get_logger().info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = self.planning_component.plan(multi_plan_parameters=multi_plan_parameters)
        elif single_plan_parameters is not None:
            plan_result = self.planning_component.plan(single_plan_parameters=single_plan_parameters)
        else:
            plan_result = self.planning_component.plan()
        self.get_logger().info("Plan result: " +  str(plan_result))
        
        return plan_result

def main(args=None):
    rclpy.init(args=args)
    node = MoveitPyServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()