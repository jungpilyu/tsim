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
    client_gp = node.create_client(GetPosition, 'get_position')
    client_rs = node.create_client(Empty, 'reset')
    ready = False
    while not ready:
        node.get_logger().info('waiting for the service availablity.')
        timeout = 1.0
        ready = client_gp.wait_for_service(timeout) and client_rs.wait_for_service(timeout)
    node.get_logger().info('service available now.')
    async def callback_gp(msg):
        nonlocal node, client_gp
        if msg.data != 'q': # 'get_position' key
            return
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
    async def callback_rs(msg):
        nonlocal node, client_rs
        if msg.data != 'r': # 'reset' key
            return
        req = Empty.Request()
        future = client_rs.call_async(req)
        node.get_logger().info('calling reset service.')
        try:
            result = await future
        except Exception as e:
            node.get_logger().info('reset service call failed: {}'.format(e))
        else:
            node.get_logger().info('Reset service call succeeded.')
            node.get_logger().info('Robot at {} to the {}'.format((result.x, result.y), result.head))

    # 3. Process node callbacks    
    subscriber_gp = node.create_subscription(String, 'keys', callback_gp, 10)
    subscriber_rs = node.create_subscription(String, 'keys', callback_rs, 10)
    rclpy.spin(node)

    # 4. Shutdown
    node.destroy_client(client_gp)
    node.destroy_client(client_rs)
    node.destroy_subscription(subscriber_gp)
    node.destroy_subscription(subscriber_rs)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()