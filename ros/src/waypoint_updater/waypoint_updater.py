#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Int32

import math
import tf

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

LOOKAHEAD_WPS = 60 # Number of waypoints we will publish. You can change this number
MAX_SPD = 20 * 0.44704 *0.8 # max speed in mps

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)
        
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        
        self.len_waypoints = 9999
        # car position
        self.pose = None
        # current timestamp
        self.pose_time_stamp = None
        # waypoints with red light
        self.red_tl_wp = None
        # waypoints separetely
        self.waypoints = None
        self.curr_velocity = None
        # whole list of waypoints
        self.full_waypoints = None
        #main loop
        self.loop()
        

        rospy.spin()

    def loop(self):
        # the main publisher loop
        rate = rospy.Rate(10) # <40hz
        
        
        # prevent errors
        while (self.pose is None) or (self.waypoints is None):
            rate.sleep()
        
        while not rospy.is_shutdown():
            
            closestWaypoint_ind = self.get_curr_waypoint_index()
            # distance between car and closest
            wp_x = self.waypoints[closestWaypoint_ind].pose.pose.position.x
            wp_y = self.waypoints[closestWaypoint_ind].pose.pose.position.x
            
            delta_x = wp_x - self.pose.position.x
            delta_y = wp_y - self.pose.position.y
            
            head = math.atan2(delta_y, delta_x)
            orientation = abs(self.yaw - head)
            
            # limit waypoints to car ability to turn
            if orientation > math.pi / 4:
                closestWaypoint_ind = closestWaypoint_ind + 1
            
            # use index as starting point
            l_start = max(0,closestWaypoint_ind-1)
            
            # create local variable with only relevant waypoints
            #waypoints = self.waypoints[l_start:l_start + self.len_final_waypoints]
            
            # print out debug info
            if(self.curr_velocity is not None):
                rospy.loginfo("Start - {}  Current Velocity - {}".format(l_start, self.red_tl_wp,self.curr_velocity.twist.linear.x))               
            
            # set up max speed
            for wp in range (LOOKAHEAD_WPS):
            #for wp, waypoint in enumerate(waypoints):
                self.set_waypoint_velocity(self.waypoints, l_start+wp, MAX_SPD)
            
            # clean up 
            if l_start + LOOKAHEAD_WPS >= self.len_waypoints:
                for wp in self.waypoints[-10:]:
                    self.set_waypoint_velocity(self.waypoints, l_start-wp, 0)
            
            # if we have information about red light
            if self.red_tl_wp != None and self.red_tl_wp > 0:
                
                # -4  - offsets car from the road
                wp_stop = self.waypoints[self.red_tl_wp-4]
                
                
                # get distance to traffic light
                distance_to_red = self.cal_distance(self.waypoints[l_start], wp_stop.pose)
                rospy.loginfo("Stop in - {}".format(distance_to_red))
                
                #if distance_to_red < 30:
                # use distance to set up waypoints
                for wp in range (LOOKAHEAD_WPS):
                #for waypoint_index, waypoint in enumerate(waypoints):
                    #distance_to_red = self.cal_distance(self.waypoints[l_start+wp], wp_stop.pose)
                
                    #velocity = math.sqrt(distance_to_red)
                    #if velocity < 15.0:
                    self.waypoints[l_start+wp].twist.twist.linear.x = 0
                    #elif velocity < self.waypoints[l_start+wp].twist.twist.linear.x:
                    #    self.waypoints[l_start+wp].twist.twist.linear.x = velocity
            '''
            #unstuck            
            if self.red_tl_wp == -1 and self.get_waypoint_velocity(self.waypoints[l_start-1]) == 0:                
                self.set_waypoint_velocity(self.waypoints, l_start-1, MAX_SPD)
                    #rospy.loginfo("no red")
            '''
            
            final_waypoints = self.get_final_waypoints()
            #rospy.loginfo("Final waypoint debug    -------- {}".format(final_waypoints))
            self.final_waypoints_pub.publish(final_waypoints)
            
            
            rate.sleep()
            
    def cal_distance(self, wp, car):
        wp_x = wp.pose.pose.position.x
        wp_y = wp.pose.pose.position.y
        wp_z = wp.pose.pose.position.z
        car_x  = car.pose.position.x
        car_y  = car.pose.position.y
        car_z  = car.pose.position.z
        return math.sqrt((wp_x-car_x)**2 + (wp_y-car_y)**2 + (wp_z-car_z)**2)
    
        
    def get_final_waypoints(self):
        
        curr_waypoint_i = self.get_curr_waypoint_index()
        
        lane = Lane()
        lane.waypoints= [self.waypoints[curr_waypoint_i+i] for i in range(LOOKAHEAD_WPS)]
        lane.header.frame_id = self.full_waypoints.header.frame_id
        lane.header.stamp = rospy.Time.now()
        return lane

    def get_curr_waypoint_index(self):
        min_i= 0
        min_d= 9999999;
        pos_x = self.pose.position.x
        pos_y = self.pose.position.y
        pos_z = self.pose.position.z
        for i, w in enumerate(self.waypoints):
            #self.cal_distance(self, w, self.pose)
        
            w_x = w.pose.pose.position.x
            w_y = w.pose.pose.position.y
            w_z = w.pose.pose.position.z
            
            distance = math.sqrt((w_x-pos_x)**2 + (w_y-pos_y)**2 + (w_z-pos_z)**2)
            if distance < min_d:
                min_d = distance
                min_i = i
                
        
        return min_i

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose
        # add timestamp        
        self.pose_time_stamp = msg.header.stamp
        
        # get yaw
        orientation_quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, 
                                  msg.pose.orientation.z, msg.pose.orientation.w]
        _,_,self.yaw = tf.transformations.euler_from_quaternion(orientation_quaternion)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints.waypoints
        self.full_waypoints = waypoints
        self.len_waypoints = len(self.waypoints)
        #self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        
        # get traffic light information
        self.red_tl_wp = msg.data
        
    def velocity_cb(self, msg):
        self.curr_velocity = msg


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
    
        # still later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        self.waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.position, waypoints[i].pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
