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
    def callback(msg):
        nonlocal node
        commands = {
            'a' : 'left turn',
            'f' : 'right turn',
            'e' : 'move forward',
            'x' : 'move backward',
        }
        act = commands.get(msg.data, 'Invalid command!')
        node.get_logger().info('Subscribing: {}'.format(act))
    # 3. Process node callbacks
    subscriber = node.create_subscription(String, 'keys', callback, 10)
    node.get_logger().info('Topic subscribed: {}'.format(subscriber.topic))
    print(""" Key - Direction mapping
            a : left turn,
            f : right turn,
            e : move forward,
            x : move backward""")
    rclpy.spin(node)
    # 4. Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()