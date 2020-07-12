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
        'a' : 'turn_left',
        'f' : 'turn_right',
        'e' : 'move_forward',
        'x' : 'move_backward',
    }
    param_dict = {v : node.declare_parameter(v, k) for k, v in commands.items()}
    new_cmds = {}
    for _, v in commands.items():
        new_key = param_dict[v].get_parameter_value().string_value
        new_cmds[new_key] = v
        # print('{} : {}'.format(new_key, v))
    commands = new_cmds

    print(commands)
    def callback(msg):
        nonlocal node
        act = commands.get(msg.data, 'Invalid command!')
        if act != 'Invalid command!':
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