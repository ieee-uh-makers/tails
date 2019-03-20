#!/usr/bin/env python

import rospy
import pigpio
from tails.msg import FlightControlSurfaces
from std_msgs.msg import Bool, String

class FCS():
    def __init__(self):
        self.state = "idle"
        self.arm_cmd = False

        rospy.init_node('fcs')

        self.armed = rospy.get_param('~armed', False)
        self.pi = pigpio.pi()

        #set pins as output and as low
        self.gpio_ail = rospy.get_param('~gpio_ail', 18)
        self.gpio_ele = rospy.get_param('~gpio_ele', 23)
        self.gpio_thr = rospy.get_param('~gpio_thr', 24)
        self.gpio_rud = rospy.get_param('~gpio_rud', 22)

        self.land_ail = rospy.get_param('~land_ail', 1500)
        self.land_ele = rospy.get_param('~land_ele', 1500)
        self.land_thr = rospy.get_param('~land_thr', 1200)
        self.land_rud = rospy.get_param('~land_rud', 1500)

        self.min_ail = rospy.get_param('~min_ail', 1000)
        self.min_ele = rospy.get_param('~min_ele', 1000)
        self.min_thr = rospy.get_param('~min_thr', 1000)
        self.min_rud = rospy.get_param('~min_rud', 1000)

        self.max_ail = rospy.get_param('~max_ail', 2000)
        self.max_ele = rospy.get_param('~max_ele', 2000)
        self.max_thr = rospy.get_param('~max_thr', 1500)
        self.max_rud = rospy.get_param('~max_rud', 2000)

        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', 10.0)

        for gpio in [self.gpio_ail, self.gpio_ele, self.gpio_thr, self.gpio_rud]:
            self.pi.set_mode(gpio, pigpio.OUTPUT)

        self.off()

        self.last_message_time = rospy.get_time() - 5.0

        rospy.Subscriber('fcs', FlightControlSurfaces,
                         self.ros_fcs_callback, queue_size=100)
        rospy.Subscriber('arm', Bool, self.ros_arm_callback, queue_size=10)
        rospy.Subscriber('state', String, self.ros_state_callback, queue_size=10)

        self.arm_pub = rospy.Publisher('arm_status', Bool, queue_size=10)

    def ros_state_callback(self, state):
        self.state = state.data

    def ros_arm_callback(self, arm):
        if self.state == "idle":
            rospy.loginfo("ARM REQUEST: %s" % arm.data)
            self.arm_cmd = arm.data
        else:
            rospy.logerr("Attempted to change ARM while in %s state" % self.state)

    def fcs(self, ail, ele, thr, rud, bypass=False):

        # Block FCS messages during arm / disarm
        if not bypass and self.state is not None and self.state == "idle" and self.arm_cmd != self.armed:
            return

        def minmax(m, a, b):
            mab = max(min(m, b), a)

            if mab != m:
                rospy.logwarn_throttle(1, "%d is outside of acceptable range [%d, %d]" % (m, a, b))

            return mab

        ail = minmax(ail, self.min_ail, self.max_ail)
        ele = minmax(ele, self.min_ele, self.max_ele)
        thr = minmax(thr, self.min_thr, self.max_thr)
        rud = minmax(rud, self.min_rud, self.max_rud)

        rospy.loginfo_throttle(1, "AIL: %f ELE: %f THR: %f RUD: %f" % (ail, ele, thr, rud))

        self.pi.set_servo_pulsewidth(self.gpio_ail, ail)
        self.pi.set_servo_pulsewidth(self.gpio_ele, ele)
        self.pi.set_servo_pulsewidth(self.gpio_thr, thr)
        self.pi.set_servo_pulsewidth(self.gpio_rud, rud)

    def off(self):
        self.fcs(1500, 1500, 1000, 1500)

    def ros_fcs_callback(self, fcs):
        if self.armed:
            ail = fcs.aileron
            ele = fcs.elevator
            thr = fcs.thrust
            rud = fcs.rudder

            self.fcs(ail, ele, thr, rud)

        else:
            rospy.logwarn_throttle(60, "Recieved FlightControlSurfaces message but not armed.")

        self.last_message_time = rospy.get_time()

    def arm(self, cmd):
        if cmd:
            self.fcs(1500, 1500, 1000, 2000, bypass=True)
        else:
            self.fcs(1500, 1500, 1000, 1000, bypass=True)

    def run(self):

        hz = 10
        r = rospy.Rate(hz)
        arm_ticks = 0

        while True:

            # Handle ARM commands in idle state
            if self.state == "idle" and self.arm_cmd != self.armed:
                if arm_ticks >= 3 * hz:
                    self.armed = self.arm_cmd
                    self.arm_pub.publish(self.armed)
                    rospy.loginfo("ARM: %s" % self.armed)
                    self.off()
                    arm_ticks = 0
                elif arm_ticks == 0:
                    self.arm(self.arm_cmd)
                    arm_ticks += 1
                else:
                    self.arm(self.arm_cmd)
                    arm_ticks += 1

            # Only allow shutdown when disarmed
            if not self.armed and rospy.is_shutdown():
                break

            # Automatically land if we are armed and lose connection to tails node
            if self.armed and rospy.get_time() - self.last_message_time > self.watchdog_timeout:

                self.fcs(self.land_ail, self.land_ele, self.land_thr, self.land_rud)

                rospy.logwarn_throttle(20, "Lost connection to tails node, landing.")

            r.sleep()

        self.off()

if __name__ == '__main__':
    f = FCS()
    f.run()
