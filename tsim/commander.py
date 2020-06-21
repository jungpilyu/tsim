"""ROS2 node to publish direction messages from keyboard.
"""

import rclpy
import sys
from std_msgs.msg import String
import select
import termios

def main():
    # A flow of typical ROS program
    # 1. Initialization
    rclpy.init()
    # 2. Create one or more nodes
    node = rclpy.create_node('commander')
    # 3. Process node callbacks
    publisher = node.create_publisher(String, 'direction', 10)
    msg = String()

    def unix_callback():
        fd = sys.stdin.fileno()
        # get the tty attributes [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        rclpy.logging.get_logger('top').info('Publishing keystrokes. Press Ctrl-C to exit.')
        old = termios.tcgetattr(fd)
        new = old[:]
        new[3] &=~ (termios.ICANON | termios.ECHO) # 3 == 'lflags'
        new[6][termios.VEOL] = 1 # 6 == 'cc'
        new[6][termios.VEOF] = 2
        termios.tcsetattr(fd, termios.TCSANOW, new)
        timeout = 0
        read_ready = select.select([sys.stdin], [], [], timeout)[0]
        if read_ready == [sys.stdin]:
            msg.data = sys.stdin.read(1)
            publisher.publish(msg)
        termios.tcsetattr(fd, termios.TCSANOW, old)

    def win_callback():
        if msvcrt.kbhit() == False:
            msg.data = msvcrt.getch()
            publisher.publish(msg)

    callback = unix_callback if USE == 'termios' else win_callback
    timer = node.create_timer(timer_period_sec = 0.2, callback = callback)
    rclpy.spin(node)
    # 4. Shutdown
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


try:
    import termios
except ImportError:
    try:
        import msvcrt
    except ImportError:
        rclpy.logging.get_logger('top').info('Import Error, I quit!')
    else:
        USE = 'msvcrt'
else:
    USE = 'termios'

if __name__ == '__main__':
    main()
