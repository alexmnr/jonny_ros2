import threading
from geometry_msgs.msg import TwistStamped
import rclpy
import pyspacemouse
import time

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (round(speed,1), round(turn,1))

def main():
    rclpy.init()

    success = pyspacemouse.open(dof_callback=pyspacemouse.print_state, button_callback=pyspacemouse.print_buttons)
    # success = pyspacemouse.open()
    if success:
        print("Space Mouse connected succesfully!")
    else:
        print("Failed to connect to Space Mouse!")

    node = rclpy.create_node('spacemouse_teleop')

    # parameters
    frame_id = node.declare_parameter('frame_id', 'link_6').value

    pub = node.create_publisher(TwistStamped, '/twist_cmds', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.1
    turn = 1.0
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
        while True:
            state = pyspacemouse.read()
            if type(state) == pyspacemouse.pyspacemouse.SpaceNavigator:
                x = state.y * -1
                y = state.x * -1
                z = state.z
                ax = state.pitch
                ay = state.roll * -1
                az = state.yaw * -1
                twist_msg.header.stamp = node.get_clock().now().to_msg()

                twist.linear.x = x * speed
                twist.linear.y = y * speed
                twist.linear.z = z * speed
                twist.angular.x = ax * turn
                twist.angular.y = ay * turn
                twist.angular.z = az * turn
                pub.publish(twist_msg)
            else:
                print("hehe")
            time.sleep(0.01)

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

if __name__ == '__main__':
    main()
