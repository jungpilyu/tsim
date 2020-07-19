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
from tsim_interfaces.action import AutoPilot
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

autopilot_cmds = {
    'request_goal' : 'u',
    'cancel_goal'  : 'p',
}

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
    
    goal_handle = None
    def goal_response_callback(future):
        nonlocal goal_handle
        goal_handle = future.result()
        if not goal_handle.accepted:
            nonlocal timer
            node.get_logger().info('Goal rejected. oTL')
            goal_handle = None
            node.destroy_timer(timer)
            timer = node.create_timer(timer_period_sec = period, callback = unix_callback)            
            return
        node.get_logger().info('Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(get_result_callback)

    def get_result_callback(future):
        nonlocal timer
        result = future.result().result.cmds
        status = future.result().status
        print(status)
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = String()        
            msg.data = result[-1]
            publisher.publish(msg)
            node.get_logger().info('Goal succeeded! ')
        else:
            node.get_logger().info('Goal failed with status code: {}'.format(status))
        node.destroy_timer(timer)
        timer = node.create_timer(timer_period_sec = period, callback = unix_callback)
       

    def feedback_callback(fb):
        node.get_logger().info('Received feedback: {}'.format(fb.feedback.partial_cmds))
        msg = String()
        msg.data = fb.feedback.partial_cmds[-1]
        publisher.publish(msg)
    
    def cancel_done(future):
        goals_canceling = future.result().goals_canceling # [action_msgs.msg.GoalInfo,...]
        if len(goals_canceling) > 0:
            nonlocal timer
            node.get_logger().info('Goal canceled')
            node.destroy_timer(timer)
            timer = node.create_timer(timer_period_sec = period, callback = unix_callback)
        else:
            node.get_logger().info('Goal not canceled')

    def unix_callback2():
        node.get_logger().info('Key \'' + autopilot_cmds['cancel_goal'] + '\' in to cancel')
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd, tty.TCSANOW) 
        timeout = 0
        read_ready = select.select([sys.stdin], [], [], timeout)[0]
        if read_ready == [sys.stdin]:
            if sys.stdin.read(1) == autopilot_cmds['cancel_goal']:
                # if goal_handle == None:
                #     node.get_logger().info('Goal unavailable to be canceled')
                #     return
                 
                node.get_logger().info('Canceling goal...')

                future = goal_handle.cancel_goal_async()
                future.add_done_callback(cancel_done)

        termios.tcsetattr(fd, termios.TCSANOW, old)

    def unix_callback():
        node.get_logger().info('Please key in!')
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd, tty.TCSANOW)
        timeout = 0
        read_ready = select.select([sys.stdin], [], [], timeout)[0]
        if read_ready == [sys.stdin]:
            msg = String()
            msg.data = sys.stdin.read(1)
            if msg.data == autopilot_cmds['request_goal']:
                nonlocal timer
                goal_msg = AutoPilot.Goal()
                goal_msg.goal_x = 0
                goal_msg.goal_y = 0
                node.get_logger().info('Sending goal request...')
                send_goal_future = action_client.send_goal_async(goal_msg,
                                        feedback_callback=feedback_callback)
                send_goal_future.add_done_callback(goal_response_callback)
                node.destroy_timer(timer)
                timer = node.create_timer(timer_period_sec = period, callback = unix_callback2)
                return

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
    
    action_client = ActionClient(node, AutoPilot, 'autopilot')
    node.get_logger().info('Waiting for action server...')
    action_client.wait_for_server()

    period = param.get_parameter_value().double_value
    timer = node.create_timer(timer_period_sec = period, callback = unix_callback)
    node.get_logger().info('Publishing keystrokes.')

    rclpy.spin(node)
    action_client.destroy()
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
