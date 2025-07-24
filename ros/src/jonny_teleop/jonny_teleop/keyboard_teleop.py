import sys
import threading

from geometry_msgs.msg import TwistStamped
import rclpy

import termios
import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Linear:
        w    r
   a         d
        s    f
Angular:
   z    u    i
   h         k
        j     

anything else : stop

y/x : increase/decrease max speeds by 0.1
c/v : increase/decrease only linear speed by 0.1
b/n : increase/decrease only angular speed by 0.1

CTRL-C to quit
"""

moveBindings = {
    "w": ( 1, 00, 00, 00, 00, 00),
    "s": (-1, 00, 00, 00, 00, 00),
    "a": (00,  1, 00, 00, 00, 00),
    "d": (00, -1, 00, 00, 00, 00),
    "r": (00, 00,  1, 00, 00, 00),
    "f": (00, 00, -1, 00, 00, 00),
    "u": (00, 00, 00,  1, 00, 00),
    "j": (00, 00, 00, -1, 00, 00),
    "h": (00, 00, 00, 00,  1, 00),
    "k": (00, 00, 00, 00, -1, 00),
    "y": (00, 00, 00, 00, 00,  1),
    "i": (00, 00, 00, 00, 00, -1),
}

speedBindings = {
    'x': (0.1, 0.1),
    'z': (-0.1, -0.1),
    'v': (0.1, 0),
    'c': (-0.1, 0),
    'n': (0, 0.1),
    'b': (0, -0.1),
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (round(speed,1), round(turn,1))

def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    frame_id = node.declare_parameter('frame_id', 'link_6').value

    pub = node.create_publisher(TwistStamped, '/twist_cmds', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 1.0
    turn = 1.0
    speed_normalizer = 0.02
    turn_normalizer = 0.3
    x = 0.0
    y = 0.0
    z = 0.0
    ax = 0.0
    ay = 0.0
    az = 0.0

    twist_msg = TwistStamped()

    twist = twist_msg.twist
    twist_msg.header.stamp = node.get_clock().now().to_msg()
    twist_msg.header.frame_id = frame_id

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0] * speed_normalizer * -1
                y = moveBindings[key][1] * speed_normalizer
                z = moveBindings[key][2] * speed_normalizer
                ax = moveBindings[key][3] * turn_normalizer * -1
                ay = moveBindings[key][4] * turn_normalizer
                az = moveBindings[key][5] * turn_normalizer
            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                turn = turn + speedBindings[key][1]
                x = 0.0
                y = 0.0
                z = 0.0
                ax = 0.0
                ay = 0.0
                az = 0.0
                print(vels(speed, turn))
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                ax = 0.0
                ay = 0.0
                az = 0.0
                if (key == '\x03'):
                    break

            twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = ax * turn
            twist.angular.y = ay * turn
            twist.angular.z = az * turn
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
