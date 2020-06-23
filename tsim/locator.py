"""ROS2 node to locate me.
"""

import rclpy
from tsim_interfaces.srv import GetPosition
from std_srvs.srv import Empty
from std_msgs.msg import String

move = {
    'a' : (0, 'turn left'),
    'f' : (0, 'turn right'),
    'e' : (1, 'move forward'),
    'x' : (-1, 'move backward'),
}
head_north = {
    'turn left' : 'west',
    'turn right' : 'east',
}
head_south = {
    'turn left' : 'east',
    'turn right' : 'west',
}
head_east = {
    'turn left' : 'north',
    'turn right' : 'south',
}
head_west = {
    'turn left' : 'south',
    'turn right' : 'north',
}

def main(args=None):
    # A flow of typical ROS program
    # 1. Initialization
    rclpy.init(args=args)
    # 2. Create one or more nodes
    node = rclpy.create_node('locator')
    head = 'north'
    robot_x = 0
    robot_y = 0
    def callback(msg):
        nonlocal node, robot_x, robot_y, head
        global move
        act = move.get(msg.data, 'Invalid command!')
        if act == 'Invalid command':
            return
        if head == 'north':
            robot_y += act[0]
            head = head_north.get(act[1], 'north')
        elif head == 'south':
            robot_y -= act[0]
            head = head_south.get(act[1], 'south')
        elif head == 'east':
            robot_x += act[0]
            head = head_east.get(act[1], 'east')
        else:
            robot_x -= act[0]
            head = head_west.get(act[1], 'west')
    def reset_callback(request, response):
        nonlocal node, robot_x, robot_y, head
        robot_x = 0
        robot_y = 0
        head = 'north'
        node.get_logger().info('Reset Position')
        return response
    def pos_callback(request, response):
        nonlocal node, robot_x, robot_y, head
        response.x = robot_x
        response.y = robot_y
        response.head = head
        node.get_logger().info('Robot at {} to the {}'.format((robot_x, robot_y), head))
        return response
    
    # 3. Process node callbacks
    node.create_subscription(String, 'keys', callback, 10)
    reset_srv = node.create_service(Empty, 'reset', reset_callback)
    pos_srv = node.create_service(GetPosition, 'get_position', pos_callback)
    node.get_logger().info('Start locator server.')
    rclpy.spin(node)
    # 4. Shutdown
    node.destroy_service(reset_srv)
    node.destroy_service(pos_srv)
    rclpy.shutdown()

if __name__ == '__main__':
    main()