"""ROS2 node to call service to locator.
"""

import rclpy
from tsim_interfaces.srv import GetPosition
from std_srvs.srv import Empty
from std_msgs.msg import String

def main(args=None):
    # A flow of typical ROS program
    # 1. Initialization
    rclpy.init(args=args)
    # 2. Create one or more nodes
    node = rclpy.create_node('requester')
    cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
    client_gp = node.create_client(GetPosition, 'get_position', callback_group=cb_group)
    client_rs = node.create_client(Empty, 'reset', callback_group=cb_group)
    ready = False
    while not ready:
        node.get_logger().info('waiting for the service availablity.')
        timeout = 1.0
        ready = client_gp.wait_for_service(timeout) and client_rs.wait_for_service(timeout)
    node.get_logger().info('service available now.')
    async def callback(msg):
        nonlocal node, client_gp, client_rs
        if msg.data == 'q': # 'get_position' key
            req = GetPosition.Request()
            future = client_gp.call_async(req)
            node.get_logger().info('calling get_position service.')
            try:
                result = await future
            except Exception as e:
                node.get_logger().info('get_position service call failed: {}'.format(e))
            else:
                node.get_logger().info('get_position service call succeeded.')
                node.get_logger().info('Robot at {} to the {}'.format((result.x, result.y), result.head))
        elif msg.data == 'r': # 'reset' key
            req = Empty.Request()
            future = client_rs.call_async(req)
            node.get_logger().info('calling reset service.')
            try:
                result = await future
            except Exception as e:
                node.get_logger().info('Reset service call failed: {}'.format(e))
            else:
                node.get_logger().info('Reset service call succeeded.')

    # 3. Process node callbacks    
    subscriber = node.create_subscription(String, 'keys', callback, 10)
    rclpy.spin(node)

    # 4. Shutdown
    node.destroy_client(client_gp)
    node.destroy_client(client_rs)
    node.destroy_subscription(subscriber)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()