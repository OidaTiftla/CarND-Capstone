#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        self.controller = Controller(
            vehicle_mass=vehicle_mass,
            fuel_capacity=fuel_capacity,
            brake_deadband=brake_deadband,
            decel_limit=decel_limit,
            accel_limit=accel_limit,
            wheel_radius=wheel_radius,
            wheel_base=wheel_base,
            steer_ratio=steer_ratio,
            max_lat_accel=max_lat_accel,
            max_steer_angle=max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # Add other member variables you need below
        self.twist = None
        self.dbw_enabled = None
        self.current_velocity = None

        self.throttle = self.brake = self.steering = 0.

        self.loop()

    def loop(self):
        wait_for_twist = None
        wait_for_dbw = None
        wait_for_current_velocity = None
        wait_dbw_enabled = None

        rospy.loginfo("[DBW] start loop at 50Hz")
        rate = rospy.Rate(50) # Hz
        while not rospy.is_shutdown():
            # Messages if not all variables are available
            if wait_for_twist is None or (wait_for_twist and self.twist is not None):
                if self.twist is None:
                    rospy.logwarn("[DBW] no twist available")
                    wait_for_twist = True
                else:
                    rospy.loginfo("[DBW] twist available")
                    wait_for_twist = False
            if wait_for_dbw is None or (wait_for_dbw and self.dbw_enabled is not None):
                if self.dbw_enabled is None:
                    rospy.logwarn("[DBW] no dbw available")
                    wait_for_dbw = True
                else:
                    rospy.loginfo("[DBW] dbw available")
                    wait_for_dbw = False
            if wait_for_current_velocity is None or (wait_for_current_velocity and self.current_velocity is not None):
                if self.current_velocity is None:
                    rospy.logwarn("[DBW] no current velocity available")
                    wait_for_current_velocity = True
                else:
                    rospy.loginfo("[DBW] current velocity available")
                    wait_for_current_velocity = False

            # Do the actual work
            if not None in (self.twist, self.dbw_enabled, self.current_velocity):
                # TODO: Get predicted throttle, brake, and steering using `twist_controller`
                # You should only publish the control commands if dbw is enabled
                current_vel = self.current_velocity.twist.linear.x
                linear_vel = self.twist.twist.linear.x
                angular_vel = self.twist.twist.angular.z
                self.throttle, self.brake, self.steering = self.controller.control(
                    current_vel,
                    self.dbw_enabled,
                    linear_vel,
                    angular_vel)

                if self.dbw_enabled:
                    if wait_dbw_enabled is None or wait_dbw_enabled:
                        rospy.loginfo("[DBW] dbw enabled")
                    wait_dbw_enabled = False
                    self.publish(self.throttle, self.brake, self.steering)
                elif wait_dbw_enabled is None or not wait_dbw_enabled:
                    wait_dbw_enabled = True
                    rospy.logwarn("[DBW] dbw disabled")
            rate.sleep()

    def publish(self, throttle, brake, steer):
        '''
        throttle: 0 to 1 = 0 to 100 %
        brake: torque in Nm (calculate it using acceleration, weight of vehicle and wheel radius)
        '''

        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def twist_cb(self, twist):
        self.twist = twist

    def dbw_enabled_cb(self, dbw_enabled):
        self.dbw_enabled = dbw_enabled.data

    def current_velocity_cb(self, current_velocity):
        self.current_velocity = current_velocity


if __name__ == '__main__':
    DBWNode()
