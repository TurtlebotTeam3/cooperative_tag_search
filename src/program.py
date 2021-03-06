#! /usr/bin/env python
import rospy
import math
import tf
import actionlib
import os
import time

from playsound import playsound

from path_planing.srv import FindPathToGoal, FindShortestPath
from path_planing.msg import FullPath, PathPoint
from tag_manager.srv import GetTags
from tag_manager.msg import TagList, TagPoint
from simple_odom.msg import CustomPose, PoseConverted
from robot_localisation.srv import Localise
from simple_camera.srv import EnableBlobDetection, EnableTagKnownCheck
from move_to_goal.srv import Move

from geometry_msgs.msg import Pose, Point, PointStamped
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, MapMetaData
from path_drive.msg import PathDriveAction, PathDriveActionGoal, PathDriveActionResult

class CooperativeTagSearch:

    def __init__(self):
        rospy.init_node('cooperative_tag_search', anonymous=True)
        self.rate = rospy.Rate(20)

        script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
        rel_path = "../sound/Transformer.mp3"
        self.sound_file_path = os.path.join(script_dir, rel_path)
        playsound(self.sound_file_path)

        self.localised = False
        self.tag_list = []
        self.tag_list_searching = []
        self.tag_list_found = []
        self.approaching = None
        self.approaching_metric = None

        self.near_tag = False

        self.waypoints = []

        self.pose = Pose()
        self.pose_converted = PoseConverted()

        self.blowUpCellNum = 3
        self.robot_radius = 2

        self.map_info = MapMetaData()

        self.tag_detection_radius = 8
        self.is_navigating = False
        self.waypointsAvailable = False

        self.no_more_tags_printed = False

        self.calculate_next = False
        
        rospy.loginfo("--- publisher ---")
        # --- Publishers ---
        self.pub_coop_tag_searching = rospy.Publisher('/coop_tag/searching', PointStamped, queue_size=1)
        self.pub_coop_tag_reached = rospy.Publisher('/coop_tag/reached', PointStamped, queue_size=1)

        rospy.loginfo("--- subscriber ---")
        # --- Subscribers ---
        self.sub_coop_tag_searching = rospy.Subscriber('/coop_tag/searching', PointStamped, self._coop_tag_searching_receive)
        self.sub_coop_tag_reached = rospy.Subscriber('/coop_tag/reached', PointStamped, self._coop_tag_reached_receive)
        self.pose_subscriber = rospy.Subscriber('simple_odom_pose', CustomPose, self._handle_update_pose)
        self.sub_goal_reached = rospy.Subscriber('move_to_tag/reached', Bool, self._goal_reached_handle)
        
        rospy.loginfo("--- service wait ---")
        # --- Service wait ---
        rospy.loginfo("1")
        rospy.wait_for_service('find_shortest_path_service')
        rospy.loginfo("2")
        rospy.wait_for_service('get_tags')
        rospy.loginfo("3")
        rospy.wait_for_service('localise_robot_service')
        rospy.loginfo("4")
        rospy.wait_for_service('enable_blob_detection_service')
        rospy.loginfo("5")
        rospy.wait_for_service('enable_tag_known_check_service')
        rospy.loginfo("6")
        rospy.wait_for_service('drive_back_and_rotate')
        
        rospy.loginfo("--- services ---")
        # --- Services ---
        self.find_shortest_path_service = rospy.ServiceProxy('find_shortest_path_service', FindShortestPath)
        self.get_tags_service = rospy.ServiceProxy('get_tags', GetTags)
        self.robot_localisation_service = rospy.ServiceProxy('localise_robot_service', Localise)
        self.enable_blob_detection_service = rospy.ServiceProxy('enable_blob_detection_service', EnableBlobDetection)
        self.enable_tag_known_check_service = rospy.ServiceProxy('enable_tag_known_check_service', EnableTagKnownCheck)
        self.drive_back_and_rotate_service = rospy.ServiceProxy('drive_back_and_rotate', Move)

        rospy.loginfo("--- action server wait ---")
        self.action_result = None
        self.client = actionlib.SimpleActionClient('path_drive_server', PathDriveAction)
        self.client.wait_for_server()

        rospy.loginfo("--- setup ---")
        self._setup()
        rospy.loginfo("--- ready ---")

    def _setup(self):
        """
        Setup method, does some magic map request and gets the resolution and offsets.
        """
        map = rospy.wait_for_message('map', OccupancyGrid)
        self.map_info = map.info

    def _coop_tag_searching_receive(self, data):
        """
        Handles messages on topic coop_tag/searching. Remove tag from the open list and add it to the searching list.
        """

        x = int(math.floor((data.point.x - self.map_info.origin.position.x)/self.map_info.resolution))
        y = int(math.floor((data.point.y - self.map_info.origin.position.x)/self.map_info.resolution))

        rospy.loginfo("Searching received - x= " + str(x) + " y= " + str(y))

        (y_known, x_known) = self._find_tag_in_list(self.tag_list, x, y)
        if y_known != None and x_known != None:
            self.tag_list.remove((y_known, x_known))
            #show tag_list list in rviz
            self.tag_list_searching.append((y_known, x_known))
            if self.approaching != None and y_known != self.approaching.point.y and x_known != self.approaching.point.x:
                rospy.loginfo("Other robot is searching for: x=" + str(x_known) + " y=" + str(y_known))
            rospy.loginfo('Searching received - Tags left: ' + str(len(self.tag_list)))

    def _coop_tag_reached_receive(self, data):
        """
        Handles messages on topic coop_tag/reached. Remove tag from searching list and add it to the reached list.
        """

        x = int(math.floor((data.point.x - self.map_info.origin.position.x)/self.map_info.resolution))
        y = int(math.floor((data.point.y - self.map_info.origin.position.x)/self.map_info.resolution))

        rospy.loginfo("Reached received - x= " + str(x) + " y= " + str(y))

        (y_known, x_known) = self._find_tag_in_list(self.tag_list_searching, x, y)
        if y_known != None and x_known != None:
            self.tag_list_searching.remove((y_known, x_known))
            self.tag_list_found.append((y_known, x_known))
            if self.approaching != None and y_known != self.approaching.point.y and x_known != self.approaching.point.x:
                rospy.loginfo("Other robot is reached: x=" + str(x_known) + " y=" + str(y_known))
            rospy.loginfo('Reached received - Tags left: ' + str(len(self.tag_list)))

    def _find_tag_in_list(self, list, x, y):
        """
        Find the tag which represents the given x and y in the given list.
        """
        for (y_known, x_known) in list:
            if x_known >= (x - self.tag_detection_radius) and x_known <= (x + self.tag_detection_radius) and y_known >= (y - self.tag_detection_radius) and y_known <= (y + self.tag_detection_radius):
                return (y_known, x_known)

        return (None, None)

    def _handle_update_pose(self, data):
        """
        Update Pose
        """
        self.pose = data.pose
        self.pose_converted = data.pose_converted
        self.robot_pose_available = True

        if self.approaching != None:
            self._check_tag()

    def _check_tag(self):
        """
        Check if robot is near the tag that needs to be approached and acitvate the blob detection.
        """
        if self.pose_converted.x >= self.approaching.point.x - 10 and self.pose_converted.x <= self.approaching.point.x + 10 and self.pose_converted.y >= self.approaching.point.y - 10 and self.pose_converted.y <= self.approaching.point.y + 10:
            if self.near_tag == False:
                rospy.loginfo("--- blob activated ---")
                bool_blob = Bool()
                bool_blob.data = True
                self.enable_blob_detection_service(bool_blob)
                bool_tag = Bool()
                bool_tag.data = False
                self.enable_tag_known_check_service(bool_tag)
            self.near_tag = True
        else:
            if self.near_tag == True:
                rospy.loginfo("--- blob deactivated ---")
                bool_blob = Bool()
                bool_blob.data = False
                self.enable_blob_detection_service(bool_blob)
                bool_tag = Bool()
                bool_tag.data = False
                self.enable_tag_known_check_service(bool_tag)
            self.near_tag = False

    def run(self):
        self.calculate_next = True
        #get the tag list
        if len(self.tag_list) == 0:
            service_response = self.get_tags_service()
            self._process_tag_list_from_service(service_response)
        while not rospy.is_shutdown():
            # start only if pose is available
            if self.robot_pose_available:
                if not self.localised:
                    # localise the robot
                    result = self.robot_localisation_service()
                    location_ok = raw_input("Localisation ok? (y for Yes / n for No) : ")
                    if location_ok == "y" or location_ok == "Y":
                        self.localised = result.localised
                    else:
                        self.localised = False
                else:
                    # find nearest tag or navigate if a goal is available
                    if not self.is_navigating:
                        if(self.waypointsAvailable == True):
                            self._navigate()
                        else:
                            #wait to give goal reach some time
                            time.sleep(8)
                            if len(self.tag_list) > 0 and self.calculate_next:
                                # robot moved to marker and there are still some more to process
                                self.calculate_next = False
                                self._calculate()
                            else:
                                if self.action_result.reached_last_goal.data:
                                    # the last way point was reached but the tag not seen
                                    # Robot moved to marker but camera did not approve arrival at marker
                                    # drive back and rotate becaus the tag must be somewhere near
                                    rospy.loginfo("start drive back and rotate")
                                    result = self.drive_back_and_rotate_service()
                                    rospy.loginfo("Move finished: " + str(result.move_finished.data)) 
                                else:
                                    if not self.calculate_next and len(self.tag_list) > 0:
                                        # driving was cancled -> recalculate the path
                                        self._calculate_again_to_last()
                                    else:
                                        if not self.no_more_tags_printed and len(self.tag_list) == 0:
                                            rospy.loginfo('--- no more tags ---')
                                            self.no_more_tags_printed = True
                                        


    def _process_tag_list_from_service(self, data):
        """
        Stores the tags received from the service localy
        """
        for point in data.tags.tags:
            self.tag_list.append((point.y, point.x))
        

    def _find_nearest_tag(self):
        """
        Get all the unprocessed tags and sends a request to the find_shortest_path service.
        """
        # create the list of goals
        goals = FullPath()
        for (y, x) in self.tag_list:
            point = PathPoint()
            point.path_x = x
            point.path_y = y
            goals.fullpath.append(point)

        # make the request
        service_response = self.find_shortest_path_service(self.blowUpCellNum, self.pose_converted.x, self.pose_converted.y, self.robot_radius, goals)
        
        # extract data
        waypoints = service_response.waypoints.fullpath
        allpoints = service_response.allPoints.fullpath

        return waypoints, allpoints, waypoints[-1]

    def _calculate(self):
        """
        Calculates the path to the nearest tag
        """
        # select tag
        self.waypoints, _ , tag_to_approach = self._find_nearest_tag()

        
        self.approaching = PointStamped()
        self.approaching.header.stamp = rospy.Time()
        self.approaching.header.frame_id = "Turtle4711/map"
        self.approaching.point.x = tag_to_approach.path_x
        self.approaching.point.y = tag_to_approach.path_y

        # convert to metric for the others
        x = (tag_to_approach.path_x * self.map_info.resolution) + self.map_info.origin.position.x
        y = (tag_to_approach.path_y * self.map_info.resolution) + self.map_info.origin.position.y

        self.approaching_metric = PointStamped()
        self.approaching_metric.header.stamp = rospy.Time()
        self.approaching_metric.header.frame_id = "Turtle4711/map"
        self.approaching_metric.point.x = x
        self.approaching_metric.point.y = y
        
        self.pub_coop_tag_searching.publish(self.approaching_metric)

        self.waypointsAvailable = True

    def _calculate_again_to_last(self):
        """
        Calculates the path to the last tag
        """
        rospy.loginfo("Recalculate path to tag")
        # create the list of goals
        goals = FullPath()
        point = PathPoint()
        point.path_x = self.approaching.point.x
        point.path_y = self.approaching.point.y
        goals.fullpath.append(point)

        # make the request
        service_response = self.find_shortest_path_service(self.blowUpCellNum, self.pose_converted.x, self.pose_converted.y, self.robot_radius, goals)
        
        # extract data
        self.waypoints = service_response.waypoints.fullpath

        self.waypointsAvailable = True

    def _navigate(self):
        """
        Navigating to the waypoints.
        """
        self.is_navigating = True

        goal = PathDriveActionGoal()

        goal.goal.waypoints.fullpath = self.waypoints

        self.client.send_goal(goal.goal)

        self.client.wait_for_result()
        
        self.action_result = self.client.get_result()

        self.waypoints = []
        self.is_navigating = False
        self.waypointsAvailable = False

    def _goal_reached_handle(self, reached):
        """
        Handles the notification that goal is reached.
        """
        
        if reached and self.approaching != None:
            # check if position is the tag position
            rospy.loginfo('--- tag reached ---')
        
            bool_blob = Bool()
            bool_blob.data = False
            self.enable_blob_detection_service(bool_blob)
            bool_tag = Bool()
            bool_tag.data = False
            self.enable_tag_known_check_service(bool_tag)

            # publish that tag reached
            self.pub_coop_tag_reached.publish(self.approaching_metric)
            
            # cancel current path
            result = self.client.get_state()

            if result == 1:
                self.client.cancel_goal()

            playsound(self.sound_file_path)

            self.approaching = None
            self.approaching_metric = None

            self.calculate_next = True
            rospy.loginfo("found it")

if __name__ == "__main__":
    try:
        cts = CooperativeTagSearch()
        cts.run()
    except rospy.ROSInterruptException:
        pass