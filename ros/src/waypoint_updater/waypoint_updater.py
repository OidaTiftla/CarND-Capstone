#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

import numpy as np
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.loop()

    def loop(self):
        wait_for_pose = None
        wait_for_waypoint_tree = None

        rospy.loginfo("[WPU] start loop at 50Hz")
        rate = rospy.Rate(50) # Hz
        while not rospy.is_shutdown():
            # Messages if not all variables are available
            if wait_for_waypoint_tree is None or (wait_for_waypoint_tree and self.waypoint_tree is not None):
                if self.waypoint_tree is None:
                    rospy.logwarn("[WPU] no waypoint tree available")
                    wait_for_waypoint_tree = True
                else:
                    rospy.loginfo("[WPU] waypoint tree available")
                    wait_for_waypoint_tree = False
            if wait_for_pose is None or (wait_for_pose and self.pose is not None):
                if self.pose is None:
                    rospy.logwarn("[WPU] no pose available")
                    wait_for_pose = True
                else:
                    rospy.loginfo("[WPU] pose available")
                    wait_for_pose = False

            # Do the actual work
            if not None in (self.pose, self.waypoint_tree):
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[(closest_idx - 1) % len(self.waypoints_2d)]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if not self.waypoint_tree:
            self.base_waypoints = waypoints
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint_idx):
        return self.base_waypoints.waypoints[waypoint_idx].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint_idx, velocity):
        self.base_waypoints.waypoints[waypoint_idx].twist.twist.linear.x = velocity

    def distance(self, wp1_idx, wp2_idx):
        dist = 0
        get_distance_func = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1_idx, wp2_idx):
            dist += get_distance_func(self.base_waypoints.waypoints[i].pose.pose.position, self.base_waypoints.waypoints[i + 1].pose.pose.position)
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
