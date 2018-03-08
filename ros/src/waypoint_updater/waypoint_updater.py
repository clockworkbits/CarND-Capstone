#!/usr/bin/env python
'''#
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''#
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
'''#
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	#rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)//[Anas]
	

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
	#self.final_waypoints_pub.publish(message)
        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
'''

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb, queue_size=1)
        #rospy.Subscriber('/obstacle_waypoints', Lane, self.obstacle_cb, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.vehiclePose = PoseStamped()
        self.base_waypoints = Lane()
        self.waypoint = Waypoint()
        self.nxtWayPoint = 0
        self.final_waypoints = Lane()
        self.seq = 0
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.vehiclePose = msg
        self.publish_final_wp()
        pass
        
    def publish_final_wp(self):
        self.nxtWayPoint = self.NextWaypoint()
        waypoints = self.base_waypoints.waypoints
        self.final_waypoints.header.seq = self.seq
        self.final_waypoints.header.stamp = rospy.get_rostime()
        #self.final_waypoints.header.frame_id #what shall i do with that!
        #TODO: cut only 200 starting from current pos 
        if (abs(self.nxtWayPoint - len(waypoints)) > LOOKAHEAD_WPS):
            self.final_waypoints.waypoints = waypoints[self.nxtWayPoint: 
                                                self.nxtWayPoint+LOOKAHEAD_WPS]
        else:
            self.final_waypoints.waypoints = waypoints[self.nxtWayPoint:]
        self.final_waypoints_pub.publish(self.final_waypoints)

        self.seq +=1 #get ready for next round

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    #From Project Path Planing main.cpp
    #https://github.com/anasmatic/CarND-Term3-Project1-Path-Planning/blob/e6ce81f7ffc1b607910966de06fc4f6527f43985/src/main.cpp
    def distance_ClosestWaypoint(self, myPos, wayPos): #x1, y1, x2, y2)
	return math.sqrt((wayPos.x-myPos.x)*(wayPos.x-myPos.x)+(wayPos.y-myPos.y)*(wayPos.y-myPos.y))

    def ClosestWaypoint(self):# x, y, maps_x, maps_y):
        closestLen = 100000000 #large number
        closestWaypoint = 0
        for i in range( len(self.base_waypoints.waypoints) ):
            #double map_x = maps_x[i];
            #double map_y = maps_y[i];
            dist = self.distance_ClosestWaypoint(
                self.vehiclePose.pose.position,
                self.base_waypoints.waypoints[i].pose.pose.position);
            #dist = distance(x,y,map_x,map_y);
            if(dist < closestLen):
                closestLen = dist
                closestWaypoint = i
        return closestWaypoint

    def NextWaypoint(self): #x, y, theta, maps_x, maps_y)
        closestWaypoint = self.ClosestWaypoint()
        map_x = self.base_waypoints.waypoints[closestWaypoint].pose.pose.position.x
        map_y = self.base_waypoints.waypoints[closestWaypoint].pose.pose.position.y
        x = self.vehiclePose.pose.position.x
        y = self.vehiclePose.pose.position.y        
        heading = math.atan2((map_y-y),(map_x-x))
        #https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        quaternion = (
            self.vehiclePose.pose.orientation.x, 
            self.vehiclePose.pose.orientation.y, 
            self.vehiclePose.pose.orientation.z, 
            self.vehiclePose.pose.orientation.w)         
        theta = euler_from_quaternion(quaternion)[2]#yaw
        angle = math.fabs(theta-heading)
        angle = min(2*math.pi - angle, angle)
        if ( angle > math.pi/4 ) :
            closestWaypoint+=1
        if (closestWaypoint == len(self.base_waypoints.waypoints) ):
            closestWaypoint = 0;
        return closestWaypoint
    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
    #I dont get it !
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')