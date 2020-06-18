"""ROS2 node to publish direction messages from keyboard.
"""

import rclpy
import sys
from std_msgs.msg import String
import select

def main():
    # A flow of typical ROS program
    # 1. Initialization
    rclpy.init()
    # 2. Create one or more nodes
    node = rclpy.create_node('remocon')
    # 3. Process node callbacks
    publisher = node.create_publisher(String, 'direction', 10)
    msg = String()

    def unix_callback():
        fd = sys.stdin.fileno()
        # get the tty attributes [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        old = termios.tcgetattr(fd)
        new = old[:]
        new[3] &= ~termios.ECHO # 3 == 'lflags'
        termios.tcsetattr(fd, termios.TCSAFLUSH, new)
        read_ready = select.select([sys.stdin], [], [], timeout = 0)[0]
        if read_ready:
            c = sys.stdin.read(1)
            msg.data = c
            publisher.publish(msg)
        termios.tcsetattr(fd, termios.TCSAFLUSH, old)

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
        USE = 'termios'
else:
    USE = 'msvcrt'

if __name__ == '__main__':
    main()
