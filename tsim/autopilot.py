"""ROS2 node to calculate autopilot sequence.
"""
import rclpy
from tsim_interfaces.action import AutoPilot
from tsim_interfaces.srv import GetPosition
import time
from rclpy.action import ActionServer
from action_msgs.msg import GoalStatus

cmd = {
        'turn_left' : 'a',
        'turn_right' : 'f',
        'move_forward' : 'e',
        'move_backward' : 'x',
    }

def main():
    # A flow of typical ROS program
    # 1. Initialization
    rclpy.init()
    # 2. Create one or more nodes
    node = rclpy.create_node('autopilot')
    cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
    client = node.create_client(GetPosition, 'get_position', callback_group=cb_group)
    ready = False
    while not ready:
        node.get_logger().info('waiting for the service availablity.')
        ready = client.wait_for_service(timeout_sec = 1.0)
    
    current_position = GetPosition.Response()
    async def execute_callback(goal_handle):
        """Generate autopilot sequence"""
        nonlocal current_position
        # align x-axis position
        goal_head = 'east' if current_position.x < goal_handle.request.goal_x else 'west'

        feedback = AutoPilot.Feedback()
        feedback.partial_cmds = []
        while current_position.head != goal_head:
            feedback.partial_cmds.append(cmd['turn_left'])
            node.get_logger().info('auto steering: turn left')
            goal_handle.publish_feedback(feedback)
            time.sleep(1)
            req = GetPosition.Request()
            current_position = await client.call_async(req)
        # if goal_handle.status == GoalStatus.STATUS_CANCELED:
        #     node.get_logger().info('Goal canceled')
        #     return AutoPilot.Result()

        while current_position.x != goal_handle.request.goal_x:
            feedback.partial_cmds.append(cmd['move_forward'])
            node.get_logger().info('auto steering: move forward')
            goal_handle.publish_feedback(feedback)
            time.sleep(1)
            req = GetPosition.Request()
            current_position = await client.call_async(req)

        goal_head = 'north' if current_position.y < goal_handle.request.goal_y else 'south'
        while current_position.head != goal_head:
            feedback.partial_cmds.append(cmd['turn_left'])
            node.get_logger().info('auto steering: turn left')
            goal_handle.publish_feedback(feedback)
            time.sleep(1)
            req = GetPosition.Request()
            current_position = await client.call_async(req)

        while current_position.y != goal_handle.request.goal_y:
            feedback.partial_cmds.append(cmd['move_forward'])
            node.get_logger().info('auto steering: move forward')
            goal_handle.publish_feedback(feedback)
            time.sleep(1)
            req = GetPosition.Request()
            current_position = await client.call_async(req)

        result = AutoPilot.Result()
        result.cmds = feedback.partial_cmds
        goal_handle.succeed()
        return result
    
    async def goal_callback(goal_request):
        """Accepts or rejects a client request to begin an action."""
        nonlocal current_position
        # request GetPosition service
        req = GetPosition.Request()
        future = client.call_async(req)
        try:
            current_position = await future
        except Exception as e:
            node.get_logger().info('Destination goal failed: {}'.format(e))
            return rclpy.action.GoalResponse.REJECT        
        node.get_logger().info('Destination goal accepted!')
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(goal_handle):
        """Accepts or rejects a client request to cancel an action."""
        node.get_logger().info('Destination goal canceled!')
        goal_handle.canceled()
        return rclpy.action.CancelResponse.ACCEPT

    # 3. Process node callbacks
    action_server = rclpy.action.ActionServer(node, AutoPilot, 'autopilot',
            execute_callback = execute_callback,
            callback_group=cb_group,
            goal_callback=goal_callback,
            cancel_callback=cancel_callback)
    node.get_logger().info('Action server in action!')

    rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    # 4. Shutdown
    action_server.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()