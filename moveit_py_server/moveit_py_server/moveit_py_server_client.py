import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_py_server_interfaces.srv import MoveToPose, MoveToJoints
import numpy as np
from sensor_msgs.msg import Joy
import tf2_ros

class MoveitPyClient(Node):
    def __init__(self):
        super().__init__('moveit_py_client')

        self.service_cb = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.cli_pose = self.create_client(MoveToPose, 'move_to_pose', callback_group=self.service_cb)
        self.cli_joints = self.create_client(MoveToJoints, 'move_to_joints', callback_group=self.service_cb)

        
        while not self.cli_pose.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service move_to_pose to become available...')
        
        while not self.cli_joints.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service move_to_joints to become available...')

        #Subscribe to the joy_action topic
        self.joy_action_sub = self.create_subscription(Joy, "/joy", self.handle_joy_action, 1, callback_group=self.service_cb)        
        self.get_logger().info('Services are available')
        self.is_on_lb = False
        self.is_on_rb = False

        #Listen to transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def handle_joy_action(self, msg):
        action = msg.buttons[4]
        if self.is_on_lb != action:
            self.is_on_lb = action
            if self.is_on_lb:
                self.get_logger().info('Moving to pose')
                target_pose = Pose()
                try:
                    current_pose = self.tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
                    target_pose.position.x = current_pose.transform.translation.x + 0.1
                    target_pose.position.y = current_pose.transform.translation.y
                    target_pose.position.z = current_pose.transform.translation.z
                    target_pose.orientation.x = current_pose.transform.rotation.x
                    target_pose.orientation.y = current_pose.transform.rotation.y
                    target_pose.orientation.z = current_pose.transform.rotation.z
                    target_pose.orientation.w = current_pose.transform.rotation.w
                except Exception as e:
                    self.get_logger().error(e)
                    return
                response_pose = self.send_move_to_pose_request(target_pose)
                # self.get_logger().info(f'MoveToPose response: success={response_pose.success}, message="{response_pose.message}"')
                return
        action = msg.buttons[5]
        if action != self.is_on_rb:
            self.get_logger().info('Moving to joints')
            self.is_on_rb = action
            if self.is_on_rb:
                target_joints = (-np.pi / 2, -np.pi * 2 / 3, np.pi * 2 / 3, -np.pi, -np.pi / 2, np.pi)
                target_joints = [float(i) for i in target_joints]
                response_joints = self.send_move_to_joints_request(target_joints)
                # self.get_logger().info(f'MoveToJoints response: success={response_joints.success}, message="{response_joints.message}"')
                return

    def send_move_to_pose_request(self, target_pose):
        req = MoveToPose.Request()
        req.target_pose = target_pose
        future_pose = self.cli_pose.call_async(req)
        future_pose.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'MoveToPose response: success={response.success}, message="{response.message}"')
        except Exception as e:
            self.get_logger().error(f'Exception while calling service: {e}')
        return response

    def send_move_to_joints_request(self, target_joints):
        req = MoveToJoints.Request()
        req.target_joints = target_joints
        future_joints = self.cli_joints.call_async(req)
        future_joints.add_done_callback(self.response_callback)

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    client = MoveitPyClient()
    rclpy.spin(client, executor=executor)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()