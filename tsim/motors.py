"""ROS2 node to subscribe direction messages
"""

import rclpy
from std_msgs.msg import String

def main():
    # A flow of typical ROS program
    # 1. Initialization
    rclpy.init()
    # 2. Create one or more nodes
    node = rclpy.create_node('motors')
    commands = {
        'a' : 'turn left',
        'f' : 'turn right',
        'e' : 'move forward',
        'x' : 'move backward',
        'r' : 'reset robot',
        'q' : 'show robot position',
    }
    print(commands)
    def callback(msg):
        nonlocal node
        act = commands.get(msg.data, 'Invalid command!')
        node.get_logger().info('Subscribing: {}'.format(act))
    # 3. Process node callbacks
    subscriber = node.create_subscription(String, 'keys', callback, 10)
    node.get_logger().info('Topic subscribed: {}'.format(subscriber.topic))
    rclpy.spin(node)
    # 4. Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()