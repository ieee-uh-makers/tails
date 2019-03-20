#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import Queue
from collections import deque
from time import time
from transitions import Machine
from transitions import State
import transitions
import numpy as np
from tails.msg import FlightControlSurfaces


class Tails():
    def __init__(self):
        rospy.init_node('tails')
        rospy.on_shutdown(self.ros_shutdown_signal)

        # Threading / Synchronization Primitives
        self.command_queue = Queue.Queue()

        self.update_rate = 1.0 / 10.0
        self.watchdog_timeout = rospy.get_param("~watchdog_timeout", 10.0)

        self.land_accel_z_mean = rospy.get_param("~land_accel_z_mean", 0.6494140625)
        self.land_accel_z_std = rospy.get_param("~land_accel_z_std", 0.01)
        self.land_accel_z_mean_percent = rospy.get_param("~land_accel_z_std", 0.05)

        self.shutdown_flag = False

        self.last_event_time = time()

        self.last_cycle_time = time()

        self.last_imu_linear = deque(maxlen=100)
        self.last_imu_angular = deque(maxlen=100)

        self.state_t = 0.

        self.linear_z_mean = None
        self.linear_z_std = None

        states = ['idle',
                  'launch',
                  'hover',
                  'land',
                  'shutdown',
                  'navigate']

        self.machine = Machine(model=self, states=states, initial='idle')

        self.machine.on_enter_idle('enter_idle')
        self.machine.on_exit_idle('exit_idle')

        self.machine.on_enter_launch('enter_launch')
        self.machine.on_exit_launch('exit_launch')

        self.machine.on_enter_hover('enter_hover')
        self.machine.on_exit_hover('exit_hover')

        self.machine.on_enter_navigate('enter_navigate')
        self.machine.on_exit_navigate('exit_navigate')

        self.machine.on_enter_land('enter_land')
        self.machine.on_exit_land('exit_land')

        self.machine.on_enter_shutdown('enter_shutdown')

        self.machine.add_transition('shutdown', 'idle', 'shutdown')
        self.machine.add_transition('launch', 'idle', 'launch')
        self.machine.add_transition('hover', 'launch', 'hover')
        self.machine.add_transition('start_navigate', 'hover', 'navigate')
        self.machine.add_transition('stop_navigate', 'navigate', 'hover')
        self.machine.add_transition('land', 'hover', 'land')
        self.machine.add_transition('grounded', 'land', 'idle')

        self.shutdown_map = {    'launch': 'hover',
                                'hover': 'land',
                                'land': 'idle',
                                'navigate': 'stop_navigate'}

        self.last_ail = 1500
        self.last_ele = 1500
        self.last_thr = 1000
        self.last_rud = 1500

        rospy.Subscriber('/cmd', String,
                         self.ros_cmd_callback, queue_size=100)

        rospy.Subscriber('/imu/data_raw', Imu, self.ros_imu_callback, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.ros_twist_callback, queue_size=10)

        self.fcs_pub = rospy.Publisher('fcs', FlightControlSurfaces, queue_size=100)
        self.state_pub = rospy.Publisher('state', String, queue_size=100)

        self.last_twist = Twist()

    def publish_fcs(self, ail=None, ele=None, thr=None, rud=None):

        fcs = FlightControlSurfaces()
        fcs.header.stamp = rospy.get_rostime()
        fcs.aileron = ail if ail is not None else self.last_ail
        fcs.elevator = ele if ele is not None else self.last_ele
        fcs.thrust = thr if thr is not None else self.last_thr
        fcs.rudder = rud if rud is not None else self.last_rud

        self.last_ail = ail if ail is not None else self.last_ail
        self.last_ele = ele if ele is not None else self.last_ele
        self.last_thr = thr if thr is not None else self.last_thr
        self.last_rud = rud if rud is not None else self.last_rud

        self.fcs_pub.publish(fcs)

    def run(self):

        self.enter_idle()

        while True:
            try:
                command = self.command_queue.get(timeout=self.update_rate).data
            except Queue.Empty:
                command = None

            # We got a new command, reset watchdog timeout
            if command is not None:
                rospy.loginfo("Recieved Command: %s" % command)
                self.last_event_time = time()

            if  self.shutdown_flag or time() - self.last_event_time > self.watchdog_timeout:
                if self.state in self.shutdown_map:
                    command = self.shutdown_map[self.state]
                    self.last_event_time = time()

            if command is not None:
                try:
                    # Attempt to execute a transition if we have one
                    if command == 'shutdown':
                        self.shutdown()
                    elif command == 'launch':
                        self.launch()
                    elif command == 'start_navigate':
                        self.start_navigate()
                    elif command == 'stop_navigate':
                        self.stop_navigate()
                    elif command == 'land':
                        self.land()
                    elif command == 'watchdog':
                        # This command only exists to keep the drone from advancing towards shutdown
                        pass
                except transitions.core.MachineError as e:
                    rospy.logwarn(str(e))

            # Execute the required states function update_rate_hz times a second
            if self.state == 'shutdown':
                return
            elif self.state == 'idle':
                self.control_idle()
            elif self.state == 'launch':
                self.control_launch()
            elif self.state == 'hover':
                self.control_hover()
            elif self.state == 'navigate':
                self.control_navigate()
            elif self.state == 'land':
                self.control_land()

            self.state_t += self.update_rate

    # Drone Control
    def control_idle(self):
        if rospy.is_shutdown():
            self.shutdown()
            return

        #5% left, 10% right, 7.5% off
        self.publish_fcs(thr=1000, rud=1500, ele=1500, ail=1500)

    def control_launch(self):
        # TODO: find optimal launch duty cycle
        self.publish_fcs(thr=1400)

        # Calls self.hover() when done
        #increase duty cycle from 0 to 100%
        if self.state_t >= 5:
            self.hover()

    def control_hover(self):
        # TODO: Figure out the duty cycle for hovering
        self.publish_fcs(thr=1300)

    def control_navigate(self):
        # Calls self.stop_navigate() when done
        # To go forward we need elevate to "go down" - throttle up
        # 7.5 to 7.6 will slowly rotate the drone right
        twist = self.last_twist

        thr = 1200 + twist.linear.z * 250
        rud = 1500 + twist.angular.z * 100
        ele = 1500 + twist.linear.x * 250
        ail = 1500 + twist.linear.y * 250

        self.publish_fcs(thr=thr, rud=rud, ele=ele, ail=ail)


    def control_land(self):
        # TODO: Figure out the duty cycle which slowly loses altitude
        self.publish_fcs(thr=1200)

        p = self.linear_z_mean >= self.land_accel_z_mean - self.land_accel_z_mean_percent*self.land_accel_z_mean
        q = self.linear_z_mean <= self.land_accel_z_mean + self.land_accel_z_mean_percent*self.land_accel_z_mean
        r = self.linear_z_std < self.land_accel_z_std

        if p and q and r:
            if self.state_t > 10.0:
                self.grounded()

    # FSM Transitions - Implement as needed
    def enter_idle(self):
        self.state_pub.publish("idle")
        rospy.loginfo("FSM: enter_idle")
        self.last_event_time = time()

        self.state_t = 0

    def exit_idle(self):
        rospy.loginfo("FSM: exit_idle")
        #does not need implementation

    def enter_launch(self):
        self.state_pub.publish("launch")
        rospy.loginfo("FSM: enter_launch")

        self.state_t = 0

    def exit_launch(self):
        rospy.loginfo("FSM: exit_launch")

    def enter_hover(self):
        self.state_pub.publish("hover")
        rospy.loginfo("FSM: enter_hover")
        self.last_event_time = time()

        self.state_t = 0

    def exit_hover(self):
        rospy.loginfo("FSM: exit_hover")

    def enter_land(self):
        self.state_pub.publish("land")
        rospy.loginfo("FSM: enter_land")
        self.last_event_time = time()

        self.state_t = 0

    def exit_land(self):
        rospy.loginfo("FSM: exit_land")

    def enter_navigate(self):
        self.state_pub.publish("navigate")
        rospy.loginfo("FSM: enter_navigate")
        self.last_event_time = time()

        self.state_t = 0

    def exit_navigate(self):
        rospy.loginfo("FSM: exit_navigate")


    def enter_shutdown(self):
        self.state_pub.publish("shutdown")
        rospy.loginfo("FSM: enter_shutdown")
        self.last_event_time = time()

        self.state_t = 0

    def ros_cmd_callback(self, cmd):
        self.command_queue.put(cmd)

    def ros_twist_callback(self, twist):
        if self.state == "navigate":
            self.last_twist = twist

    def ros_imu_callback(self, imu):
        self.last_imu_linear.append([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z])
        self.last_imu_angular.append([imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z])

        angular = np.array(self.last_imu_angular)
        linear = np.array(self.last_imu_linear)

        self.linear_z_mean = np.mean(linear[:, 2])
        self.linear_z_std = np.std(linear[:, 2])

    # ROS Signals
    def ros_shutdown_signal(self):
        self.shutdown_flag = True


if __name__ == '__main__':
    t = Tails()
    t.run()
