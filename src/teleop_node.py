import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import curses
import time
import math


class Teleop():
    def __init__(self):
        rospy.init_node('teleop')

        self.cmd_pub = rospy.Publisher('/cmd', String, queue_size=100)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.arm_pub = rospy.Publisher('/arm', Bool, queue_size=10)

        rospy.Subscriber('state', String, self.ros_state_callback, queue_size=10)
        rospy.Subscriber('arm_status', Bool, self.ros_arm_callback, queue_size=10)

        self.state = "None"
        self.armed = False


    def ros_state_callback(self, state):
        self.state = state

    def ros_arm_callback(self, armed):
        self.armed = armed

    def run(self):

        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(1)

        stdscr.timeout(100)

        last_cmd = time.time()

        key = ''
        while not rospy.is_shutdown() and key != ord('q'):
            stdscr.refresh()

            key = stdscr.getch()

            stdscr.addstr(0, 0, "Arm: %s" % self.armed)
            stdscr.addstr(0, 12, "State: %s" % self.state)

            if key == -1:
                if time.time() - last_cmd > 0.5:
                    twist = Twist()
                    self.twist_pub.publish(twist)
                continue

            stdscr.clear()

            if key == curses.KEY_UP:
                stdscr.addstr(1, 0, "Last CMD: Up")

                twist = Twist()
                twist.linear.x = 0.1
                self.twist_pub.publish(twist)

            elif key == curses.KEY_DOWN:
                stdscr.addstr(1, 0, "Last CMD: Down")

                twist = Twist()
                twist.linear.x = -0.1
                self.twist_pub.publish(twist)

            elif key == curses.KEY_LEFT:
                stdscr.addstr(1, 0, "Last CMD: Left")

                twist = Twist()
                twist.linear.y = 0.1
                self.twist_pub.publish(twist)

            elif key == curses.KEY_RIGHT:
                stdscr.addstr(1, 0, "Last CMD: Right")

                twist = Twist()
                twist.linear.y = -0.1
                self.twist_pub.publish(twist)

            elif chr(key) == 'l':
                stdscr.addstr(1, 0, "Last CMD: Launch")
                self.cmd_pub.publish("launch")
            elif chr(key) == "'":
                stdscr.addstr(1, 0, "Last CMD: Land")
                self.cmd_pub.publish("land")
            elif chr(key) == "n":
                stdscr.addstr(1, 0, "Last CMD: Start Navigate")
                self.cmd_pub.publish("start_navigate")
            elif chr(key) == ",":
                stdscr.addstr(1, 0, "Last CMD: Stop Navigate")
                self.cmd_pub.publish("stop_navigate")
            elif chr(key) == "a":
                stdscr.addstr(1, 0, "Last CMD: ARM")
                self.arm_pub.publish(True)
            elif chr(key) == "d":
                stdscr.addstr(1, 0, "Last CMD: DISARM")
                self.arm_pub.publish(False)
            elif chr(key) == "-":
                stdscr.addstr(1, 0, "Last CMD: Throttle--")

                twist = Twist()
                twist.linear.z = -0.1
                self.twist_pub.publish(twist)

            elif chr(key) == "=":
                stdscr.addstr(1, 0, "Last CMD: Throttle++")

                twist = Twist()
                twist.linear.z = 0.1
                self.twist_pub.publish(twist)

            elif chr(key) == "[":
                stdscr.addstr(1, 0, "Last CMD: Rotate Left")

                twist = Twist()
                twist.angular.z = math.pi / 4.
                self.twist_pub.publish(twist)

            elif chr(key) == "]":
                stdscr.addstr(1, 0, "Last CMD: Rotate Right")

                twist = Twist()
                twist.angular.z = -math.pi / 4.
                self.twist_pub.publish(twist)


            last_cmd = time.time()

        curses.endwin()

if __name__ == "__main__":
    teleop = Teleop()
    teleop.run()
