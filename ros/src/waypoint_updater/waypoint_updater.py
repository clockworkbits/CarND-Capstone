#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport#anas
from std_msgs.msg import Int32

import time
from tf.transformations import euler_from_quaternion
from scipy.spatial.distance import euclidean
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''
#anas_logic
WAYPOINTS_TO_BREAK = 30# how many waypoints to set velocity to 0 , before a red traffic light
#anas_logic_end
LOOKAHEAD_WPS   = 200   # Number of waypoints we will publish. You can change this
REFRESH_RATE_HZ = 2     # Number of times we update the final waypoints per second
UPDATE_MAX_ITER = 50    # Max number of iterations before considering relooking for the next waypoint in full path
DEBUG_MODE      = True # Switch for whether debug messages are printed.

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

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG if DEBUG_MODE else rospy.ERROR)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #anas_test
        self.brake = 0.0###
        #rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cb)###
        #anas_test_end
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.next_waypoint = 0
        self.previous_pos = None
        self.static_waypoints = None
        self.backup_waypoints = None#anas
        self.waiting_for_red_tl = False#anas
        self.index_start_set_all_to_zero = 0#anas
        self.index_end_set_all_to_zero = 0#anas
        self.last_update = time.time()
        
        self.next_red_tl_wp = -1
        
        rospy.spin()

    def pose_cb(self, msg):
        self.previous_pos = msg.pose
        if time.time() - self.last_update > 1. / REFRESH_RATE_HZ:
            self.publish_update()

    def waypoints_cb(self, waypoints):
        self.static_waypoints = waypoints# Lane message is a structure with header and waypoints 
        import copy
        self.backup_waypoints = copy.deepcopy(waypoints)#anas
        #rospy.loginfo('Received {} base waypoints'.format(len(self.static_waypoints.waypoints)))


    def traffic_cb(self, msg_Int32):#anas
        if self.next_red_tl_wp != msg_Int32.data :
            self.next_red_tl_wp = msg_Int32.data
            if self.waypoint_is_ahead(self.static_waypoints.waypoints[self.next_red_tl_wp]) == False:
                rospy.logdebug("waypoint is NOT ahead pos")
                self.next_red_tl_wp = -1    
            rospy.logdebug("next tl: %i",self.next_red_tl_wp)
        pass

    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance_to_previous(self, position):
        return euclidean(get_position(self.previous_pos), get_position(position))

    def waypoint_is_ahead(self, waypoint):
        w_x, w_y, _ = get_position(waypoint.pose.pose)
        p_x, p_y, _ = get_position(self.previous_pos)
        heading = math.atan2(w_y-p_y, w_x-p_x)
        yaw = euler_from_quaternion(get_orientation(self.previous_pos))[2]
        angle = normalize_angle(yaw-heading)
        return True if math.fabs(angle) < math.pi/4. else False

    def update_next_waypoint(self):
        it = 0
        nb_waypoints = len(self.static_waypoints.waypoints)
        while not self.waypoint_is_ahead(self.static_waypoints.waypoints[self.next_waypoint % nb_waypoints]) and \
                it < UPDATE_MAX_ITER:
            it += 1
            self.next_waypoint += 5  # We look at the next one
            if self.next_waypoint > len(self.static_waypoints.waypoints) - 1:
                self.next_waypoint = 0

        # Searching the next waypoint in the full path takes much longer, we want to avoid it as much as possible
        if it == UPDATE_MAX_ITER:
            self.search_next_waypoint()

    def search_next_waypoint(self):
        self.next_waypoint = 0
        rospy.logwarn("Initiating search for closest waypoint...")
        # We basically search among all static waypoints the closest waypoint ahead
        for i in range(len(self.static_waypoints.waypoints)):
            if self.waypoint_is_ahead(self.static_waypoints.waypoints[i]) and \
                    self.distance_to_previous(self.static_waypoints.waypoints[i].pose.pose) < \
                    self.distance_to_previous(self.static_waypoints.waypoints[self.next_waypoint].pose.pose):
                self.next_waypoint = i
        rospy.logwarn('Found next closest waypoint: {}'.format(self.next_waypoint))


    def next_waypoints_circular(self):
        wp = self.next_waypoint
        #anas_logic_begin ~~~~~~~~~~~~~~~~~
        if self.next_red_tl_wp > -1 and (wp+WAYPOINTS_TO_BREAK) >= self.next_red_tl_wp :
            if self.waiting_for_red_tl == False:
                self.index_start_set_all_to_zero = wp
                self.index_end_set_all_to_zero = self.next_red_tl_wp
                for i in range(wp, self.next_red_tl_wp):
                    self.set_waypoint_velocity(self.static_waypoints.waypoints,i,0)
                self.waiting_for_red_tl = True
                rospy.logdebug("set everything to 0 @ %i to %i",wp,self.next_red_tl_wp)
        else :
            if self.waiting_for_red_tl == True :
                self.waiting_for_red_tl = False
                for i in range(self.index_start_set_all_to_zero,self.index_end_set_all_to_zero):
                    self.set_waypoint_velocity(self.static_waypoints.waypoints,i,
                            self.get_waypoint_velocity(self.backup_waypoints.waypoints[i]))
                rospy.logdebug("GREEN, donot wait, reset %i to %i",self.index_start_set_all_to_zero,self.index_end_set_all_to_zero)
        #anas_logic_end ~~~~~~~~~~~~~~~~~~~~~~~
        if wp + LOOKAHEAD_WPS < len(self.static_waypoints.waypoints) - 1:
            return self.static_waypoints.waypoints[wp:wp + LOOKAHEAD_WPS]
        else:
            return self.static_waypoints.waypoints[wp:] + \
                   self.static_waypoints.waypoints[0:wp + LOOKAHEAD_WPS - len(self.static_waypoints.waypoints) + 1]
    #anas_test
    def brake_cb(self, msg):###
        if self.brake != msg.pedal_cmd:###
            self.brake = msg.pedal_cmd###
            rospy.logdebug("brake::::: %f",msg.pedal_cmd)###
    #anas_test_end
    def publish_update(self):
        # Emits the new waypoints, but only if we have received the base waypoints
        if self.static_waypoints:
            self.update_next_waypoint()
            #rospy.logdebug("Next waypoint: {}".format(self.next_waypoint))
            self.final_waypoints_pub.publish(Lane(waypoints=self.next_waypoints_circular()))

        self.last_update = time.time()


if __name__ == '__main__':
    print 'Starting waypoint_updater...'
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
