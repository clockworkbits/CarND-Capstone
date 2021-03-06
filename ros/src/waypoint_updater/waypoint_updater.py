#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightArray, TrafficLight
from std_msgs.msg import Int32
import yaml
import time
from tf.transformations import euler_from_quaternion
from scipy.spatial.distance import euclidean
import math
from itertools import cycle, islice
import timeit
from scipy.spatial import KDTree
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS   = 200   # Number of waypoints we will publish. For Test Lot, please put a value smaller than 60
CIRCULAR_WPS    = False # If True, assumes that the path to follow is a loop
REFRESH_RATE_HZ = 50     # Number of times we update the final waypoints per second should be 50Hz for submission
DEBUG_MODE      = False # Switch for whether debug messages are printed.
TL_DETECTOR_ON  = True  # If False, switches to direct traffic light subscription
DECELLERATION   = 3     # Decelleration in m/s^2


def normalize_angle(angle):
    if angle > math.pi:
        return angle - 2 * math.pi
    elif angle < -math.pi:
        return angle + 2 * math.pi
    else:
        return angle


def get_position(pos):
    return pos.position.x, pos.position.y, pos.position.z


def get_orientation(pos):
    return pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w


def euclidean_distance(wp1, wp2):
    return euclidean(get_position(wp1.pose.pose), get_position(wp2.pose.pose))


def step_by_step_distance(waypoints, wp1, wp2):
    dist = 0
    dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
    for i in range(wp1, wp2 + 1):
        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
        wp1 = i
    return dist


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG if DEBUG_MODE else rospy.WARN)

        if DEBUG_MODE:
            rospy.logwarn('waypoint_updater: DEBUG mode enabled - Disable for submission')

        if not TL_DETECTOR_ON:
            rospy.logwarn('waypoint_updater: Traffic Light Detector disabled - Enable for submission')

        if REFRESH_RATE_HZ < 50:
            rospy.logwarn('waypoint_updater: REFRESH_RATE_HZ is lower than 50Hz - Increase for submission')

        self.next_waypoint_indices = self.next_waypoint_indices_circular if CIRCULAR_WPS else self.next_waypoint_indices_simple

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        if TL_DETECTOR_ON:
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        else:
            config_string = rospy.get_param("/traffic_light_config")
            self.stop_line_config = yaml.load(config_string)
            self.stop_line_positions = self.stop_line_config['stop_line_positions']
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_array_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.next_waypoint = 0
        self.next_red_tl_wp = 0
        self.previous_pos = None
        self.waypoint_tree = None
        self.waypoints_2d = None
        self.static_waypoints = None
        self.static_velocities = None  # Static waypoints with adjusted velocity
        self.last_update = time.time()
        self.loop()

    def loop(self):
        rate = rospy.Rate(REFRESH_RATE_HZ)
        while not rospy.is_shutdown():
            if self.previous_pos:
                self.publish_update()
            rate.sleep()

    def pose_cb(self, msg):
        self.previous_pos = msg.pose

    def waypoints_cb(self, waypoints):
        self.static_waypoints = waypoints.waypoints  # Lane message is a structure with header and waypoints
        self.static_velocities = [self.get_static_waypoint_id_velocity(wp)
                                  for wp in range(len(self.static_waypoints))]

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        rospy.loginfo('Received {} base waypoints'.format(len(self.static_waypoints)))

    def traffic_light_array_cb(self, msg):
        if self.static_waypoints and time.time() - self.last_update > 1. / REFRESH_RATE_HZ:
            tl_array = [l for l in msg.lights if self.waypoint_is_ahead(l)
                        and l.state != 2 and self.distance_to_previous(l.pose.pose) < 500]
            sorted_tl_array = sorted(tl_array, key=lambda x: self.distance_to_previous(x.pose.pose))

            # Only if we do have a red light not too far ahead, we publish a waypoint
            if len(sorted_tl_array):
                closest_tl = sorted_tl_array[0]
                tl_x, tl_y = closest_tl.pose.pose.position.x, closest_tl.pose.pose.position.y
                next_wps = self.next_waypoint_indices_simple()

                # Get corresponding stop line closest to traffic lght
                corresponding_stop_line = sorted(self.stop_line_positions,
                                          key=lambda x: euclidean(x, (tl_x, tl_y)))[0]

                # Get corresponding waypoint closest to stop line
                corresponding_wp = sorted(next_wps,
                                          key=lambda x: euclidean(corresponding_stop_line,
                                                                  (self.static_waypoints[x].pose.pose.position.x,
                                                                  self.static_waypoints[x].pose.pose.position.y)))[0]
                self.traffic_cb(Int32(corresponding_wp))
            else:
                self.traffic_cb(Int32(-1))

    def traffic_cb(self, msg):
        if self.next_red_tl_wp != msg.data:
            rospy.logdebug("===> waypoint_updater: Next Traffic Light: %i", msg.data)
            self.next_red_tl_wp = msg.data

            if self.next_red_tl_wp > 0:
                rospy.logwarn("waypoint_updater: Stop command received for waypoint  %i (ie light is red/yellow)", self.next_red_tl_wp)
            else:
                rospy.logwarn("waypoint_updater: Stop command cancelled (ie light is green/unknown)")

            # 1: restore original speeds
            self.restore_all_velocities()
            # 2: gradually reduce speed in order to get to zero to next red light
            if self.next_red_tl_wp > 0:
                wp_indices = self.next_waypoint_indices()
                first_slow_wp_set = False
                first_slow_wp = -1
                last_slow_wp = wp_indices[-1]
                for wp in wp_indices:
                    tl_wp = self.static_waypoints[self.next_red_tl_wp]
                    check_wp = self.static_waypoints[wp]
                    if self.distance_to_previous(check_wp.pose.pose) > self.distance_to_previous(tl_wp.pose.pose) - 3.0: #center of car should stop 3 meters before stop line
                        self.set_waypoint_id_velocity(wp, 0.0)
                        if wp < last_slow_wp:
                            last_slow_wp = wp
                    else:
                        dist_wp_to_stop = euclidean_distance(check_wp, tl_wp)
                        current_wp_vel = self.get_static_waypoint_id_velocity(wp)
                        # Assuming car goes at 25 mph, i.e. 11.2 m/s, we need 30 meters to stop in order to ensure we
                        # stay below 5 m/s^2
                        target_vel = min(dist_wp_to_stop / (current_wp_vel / DECELLERATION), current_wp_vel)
                        if (target_vel < current_wp_vel and not first_slow_wp_set):
                            first_slow_wp = wp
                            first_slow_wp_set = True
                        self.set_waypoint_id_velocity(wp, target_vel)

                rospy.logwarn("waypoint_updater: decreasing velocity from target speed %d to 0 between waypoints %d and %d",self.static_velocities[first_slow_wp],first_slow_wp,last_slow_wp)
            else:
                rospy.logwarn("waypoint_updater: resetting velocity targets to target speed %d",self.static_velocities[self.next_waypoint])

    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_static_waypoint_id_velocity(self, waypoint_id):
        return self.static_waypoints[waypoint_id].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def set_waypoint_id_velocity(self, waypoint_id, velocity):
        self.static_waypoints[waypoint_id].twist.twist.linear.x = velocity

    def restore_waypoint_id_velocity(self, waypoint_id):
        self.set_waypoint_id_velocity(waypoint_id, self.static_velocities[waypoint_id])

    def restore_all_velocities(self):
        for wp_id in range(len(self.static_waypoints)):
            self.restore_waypoint_id_velocity(wp_id)

    def distance_to_previous(self, position):
        return euclidean(get_position(self.previous_pos), get_position(position))

    def waypoint_is_ahead(self, waypoint):
        w_x, w_y, _ = get_position(waypoint.pose.pose)
        p_x, p_y, _ = get_position(self.previous_pos)
        heading = math.atan2(w_y-p_y, w_x-p_x)
        yaw = euler_from_quaternion(get_orientation(self.previous_pos))[2]
        angle = normalize_angle(yaw-heading)
        return True if math.fabs(angle) < math.pi/4. else False

    def search_next_waypoint(self):
        start_time = timeit.default_timer()
        rospy.logdebug("Initiating search for closest waypoint...")
        # We basically search among all static waypoints the closest waypoint ahead
        x, y, _ = get_position(self.previous_pos)
        closest_index = self.waypoint_tree.query([x, y], 1)[1]
        
        closest_coord = self.waypoints_2d[closest_index]

        prev_coord = self.waypoints_2d[closest_index - 1 if closest_index > 0 else closest_index]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vext = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vext - cl_vect)

        if val > 0:
            self.next_waypoint = (closest_index + 1) % len(self.waypoints_2d)
        else:
            self.next_waypoint = closest_index


        elapsed = timeit.default_timer() - start_time
        rospy.logdebug('Found next closest waypoint: {} - elapsed time = {}'.format(self.next_waypoint, elapsed))

    # --- Next waypoint indices functions ---
    def next_waypoint_indices_circular(self):
        cyc = cycle(range(len(self.static_waypoints)))
        return list(islice(cyc, self.next_waypoint, self.next_waypoint + LOOKAHEAD_WPS))

    def next_waypoint_indices_simple(self):
        return list(islice(range(len(self.static_waypoints)), self.next_waypoint, self.next_waypoint+LOOKAHEAD_WPS))
    # ----------------------------------------

    def next_waypoints(self):
        indices = self.next_waypoint_indices()
        return [self.static_waypoints[i] for i in indices]

    def publish_update(self):
        # Emits the new waypoints, but only if we have received the base waypoints
        if self.static_waypoints and self.waypoints_2d and self.waypoint_tree:
            self.search_next_waypoint()
            rospy.logdebug("Next waypoint: {}".format(self.next_waypoint))
            self.final_waypoints_pub.publish(Lane(waypoints=self.next_waypoints()))

        self.last_update = time.time()


if __name__ == '__main__':
    print 'Starting waypoint_updater...'
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
