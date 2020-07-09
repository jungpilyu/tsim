"""ROS2 node to publish direction messages from keyboard.
"""
import rclpy
import sys
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import FloatingPointRange
import select
import termios
import tty

def main():
    # A flow of typical ROS program
    # 1. Initialization
    rclpy.init()
    # 2. Create one or more nodes
    node = rclpy.create_node('commander')
    fp_range = FloatingPointRange(
        from_value = 0.2,
        to_value = 2.0,
        step = 0.1
    )
    pd = ParameterDescriptor(
        name = 'parameter in commander',
        type = ParameterType.PARAMETER_DOUBLE,
        description = 'set key-in period in seconds',
        read_only = False,
        floating_point_range = [fp_range]
    )
    param = node.declare_parameter('key_in_period', value = 0.2, 
                        descriptor = pd, ignore_override=False)
    # 3. Process node callbacks
    publisher = node.create_publisher(String, 'keys', 10)
    
    def unix_callback():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd, tty.TCSANOW) 
        timeout = 0
        node.get_logger().info('Please key in!')
        read_ready = select.select([sys.stdin], [], [], timeout)[0]
        if read_ready == [sys.stdin]:
            msg = String()
            msg.data = sys.stdin.read(1)
            publisher.publish(msg)
        termios.tcsetattr(fd, termios.TCSANOW, old)

    def before_parameter(param_list: [rclpy.parameter.Parameter]):
        nonlocal timer
        p_list = [its for its in param_list if its.name == 'key_in_period']
        period = p_list[0].get_parameter_value().double_value
        node.destroy_timer(timer)
        timer = node.create_timer(timer_period_sec = period, callback = unix_callback)
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(before_parameter)
    
    def after_parameter(msg: ParameterEvent):
        if msg.node != '/commander':
            return
        param_msg_list = [its for its in msg.changed_parameters if its.name == 'key_in_period']
        name = param_msg_list[0].name
        value = param_msg_list[0].value.double_value
        node.get_logger().info(
            'parameter \'{}\' = {} sec.'.format(name, value))
    node.create_subscription(ParameterEvent, 'parameter_events', after_parameter, 10)

    period = param.get_parameter_value().double_value
    timer = node.create_timer(timer_period_sec = period, callback = unix_callback)
    node.get_logger().info('Publishing keystrokes.')

    rclpy.spin(node)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
