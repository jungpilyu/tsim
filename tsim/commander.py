"""ROS2 node to publish direction messages from keyboard.
"""
import rclpy
USE = 'termios'
try:
    import termios
except ImportError:
    try:
        import msvcrt
    except ImportError:
        rclpy.logging.get_logger('top').info('Import Error, I quit!')
    else:
        USE = 'msvcrt'
import sys
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult
import select
import termios
import tty

def main():
    # A flow of typical ROS program
    # 1. Initialization
    rclpy.init()
    # 2. Create one or more nodes
    node = rclpy.create_node('commander')
    pd = ParameterDescriptor(
        name = 'parameter in commander',
        type = ParameterType.PARAMETER_DOUBLE,
        description = 'set key-in period in seconds',
        read_only = False,
#        floating_point_range = 0.1, # from_value, to_value, step
    )
    node.declare_parameter('key-in-period', value=0.2,
        descriptor=pd, ignore_override=False)
    # 3. Process node callbacks
    publisher = node.create_publisher(String, 'keys', 10)
    
    def unix_callback():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd) # get the tty attributes
        tty.setcbreak(fd, tty.TCSANOW) 
        timeout = 0
        node.get_logger().info('Please key!')
        read_ready = select.select([sys.stdin], [], [], timeout)[0]
        if read_ready == [sys.stdin]:
            msg = String()
            msg.data = sys.stdin.read(1)
            publisher.publish(msg)
        termios.tcsetattr(fd, termios.TCSANOW, old)
    def win_callback():
        if msvcrt.kbhit() == False:
            msg = String()
            msg.data = msvcrt.getch()
            publisher.publish(msg)
    def on_parameter(param_list):
        nonlocal timer
        period = param_list[0].get_parameter_value().double_value
        node.destroy_timer(timer)
        timer = node.create_timer(timer_period_sec = period, callback = callback)
        return SetParametersResult(successful=True)

    callback = unix_callback if USE == 'termios' else win_callback
    period = node.get_parameter('key-in-period').get_parameter_value().double_value
    node.add_on_set_parameters_callback(on_parameter)
    timer = node.create_timer(timer_period_sec = period, callback = callback)
    node.get_logger().info('Publishing keystrokes.')

    rclpy.spin(node)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
