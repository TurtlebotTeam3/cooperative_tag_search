import rospy
import math
import tf

from path_planing.srv import FindPathToGoal, FindShortestPath
from path_planing.msg import FullPath, PathPoint
from tag_manager.srv import GetTags
from tag_manager.msg import TagList, TagPoint
from simple_odom.msg import CustomPose, PoseConverted
from robot_localisation.srv import Localise
from simple_camera.srv import EnableBlobDetection, EnableTagKnownCheck

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, MapMetaData

class CooperativeTagSearch:

    def __init__(self):
        rospy.init_node('cooperative_tag_search', anonymous=True)
        self.rate = rospy.Rate(20)

        self.localised = False
        self.tag_list = []
        self.tag_list_searching = []
        self.tag_list_found = []
        self.approaching = None
        self.near_tag = False

        self.waypoints = []

        self.pose = Pose()
        self.pose_converted = PoseConverted()

        self.blowUpCellNum = 3
        self.robot_radius = 1

        self.map_info = MapMetaData()

        self.tag_detection_radius = 8
        self.is_navigating = False
        self.waypointsAvailable = False

        self.no_more_tags_printed = False
        
        print("--- publisher ---")
        # --- Publishers ---
        self.pub_coop_tag_searching = rospy.Publisher('coop_tag/searching', Point,queue_size=1)
        self.pub_coop_tag_reached = rospy.Publisher('coop_tag/reached', Point,queue_size=1)
        self.pub_goal = rospy.Publisher('move_to_goal/goal', Pose, queue_size=1)

        print("--- subscriber ---")
        # --- Subscribers ---
        self.sub_coop_tag_searching = rospy.Subscriber('coop_tag/searching', Point, self._coop_tag_searching_receive)
        self.sub_coop_tag_reached = rospy.Subscriber('coop_tag/reached', Point, self._coop_tag_reached_receive)
        self.pose_subscriber = rospy.Subscriber('simple_odom_pose', CustomPose, self._handle_update_pose)
        self.sub_goal_reached = rospy.Subscriber('move_to_goal/reached', Bool, self._goal_reached_handle)
        
        print("--- service wait ---")
        # --- Service wait ---
        print("1")
        rospy.wait_for_service('find_shortest_path_service')
        print("2")
        rospy.wait_for_service('get_tags')
        print("3")
        rospy.wait_for_service('localise_robot_service')
        print("4")
        rospy.wait_for_service('enable_blob_detection_service')
        print("5")
        rospy.wait_for_service('enable_tag_known_check_service')
        
        print("--- services ---")
        # --- Services ---
        self.find_shortest_path_service = rospy.ServiceProxy('find_shortest_path_service', FindShortestPath)
        self.get_tags_service = rospy.ServiceProxy('get_tags', GetTags)
        self.robot_localisation_service = rospy.ServiceProxy('localise_robot_service', Localise)
        self.enable_blob_detection_service = rospy.ServiceProxy('enable_blob_detection_service', EnableBlobDetection)
        self.enable_tag_known_check_service = rospy.ServiceProxy('enable_tag_known_check_service', EnableTagKnownCheck)


        print("--- setup ---")
        self._setup()
        print("--- ready ---")

    def _setup(self):
        """
        Setup method, does some magic map request and gets the resolution and offsets.
        """
        map = rospy.wait_for_message('map', OccupancyGrid)
        self.map_info = data.info

    def _coop_tag_searching_receive(self, data):
        """
        Handles messages on topic coop_tag/searching
        """
        (y_known, x_known) = self._find_tag_in_list(self.tag_list, int(data.x), int(data.y))
        self.tag_list.remove((y_known, x_known))
        #show tag_list list in rviz
        self.tag_list_searching.append((y_known, x_known))

    def _coop_tag_reached_receive(self, data):
        """
        Handles messages on topic coop_tag/reached
        """
        (y_known, x_known) = self._find_tag_in_list(self.tag_list_searching, int(data.x), int(data.y))
        self.tag_list_searching.remove((y_known, x_known))
        self.tag_list_found.append((y_known, x_known))

    def _find_tag_in_list(self, list, x, y):
        """
        Find the tag which represents the given x and y in the given list
        """
        for (y_known, x_known) in list:
            if x_known >= (x - self.tag_detection_radius) and x_known <= (x + self.tag_detection_radius) and y_known >= (y - self.tag_detection_radius) and y_known <= (y + self.tag_detection_radius):
                return (y_known, x_known)

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
        if self.pose_converted.x >= self.approaching.x - 10 and self.pose_converted.x <= self.approaching.x + 10 and self.pose_converted.y >= self.approaching.y - 10 and self.pose_converted.y <= self.approaching.y + 10:
            if self.near_tag == False:
                print("--- blob activated ---")
                bool_blob = Bool()
                bool_blob.data = True
                self.enable_blob_detection_service(bool_blob)
                bool_tag = Bool()
                bool_tag.data = False
                self.enable_tag_known_check_service(bool_tag)
            self.near_tag = True
        else:
            if self.near_tag == True:
                print("--- blob deactivated ---")
                bool_blob = Bool()
                bool_blob.data = False
                self.enable_blob_detection_service(bool_blob)
                bool_tag = Bool()
                bool_tag.data = False
                self.enable_tag_known_check_service(bool_tag)
            self.near_tag = False

    def run(self):
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
                    self.localised = result.localised
                else:
                    # find nearest tag or navigate if a goal is available
                    if not self.is_navigating:
                        if(self.waypointsAvailable == True):
                            self._navigate()
                        else:
                            if len(self.tag_list) > 0:
                                self._calculate()
                            else:
                                if not self.no_more_tags_printed:
                                    self.no_more_tags_printed = True
                                    print('--- no more tags ---')


    def _process_tag_list_from_service(self, data):
        """
        Stores the tags received from the service localy
        """
        for point in data.tags.tags:
            self.tag_list.append((point.y, point.x))
        #TODO
        #show tag_list list in rviz
        

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

        # send to topic
        self.approaching = Point()
        self.approaching.x = tag_to_approach.path_x
        self.approaching.y = tag_to_approach.path_y
        
        self.pub_coop_tag_searching.publish(self.approaching)

        #TODO
        #show all waypoints in rviz
        self.waypointsAvailable = True

    def _navigate(self):
        """
        Navigating to the waypoints
        """
        # get waypoint and start moving towards it
        # when success the process next
        self.is_navigating = True

        if(len(self.waypoints) > 0):
            #print(self.waypoints)
            point = self.waypoints.pop(0)
            x = point.path_x
            y = point.path_y

            #print(self.waypoints)

            # -- move to goal --
            self._move(x, y)

        else:
            self.is_navigating = False
            self.waypointsAvailable = False

    def _move(self, x, y):
        """
        Moves the rob2t to a place defined by coordinates x and y.
        """
        print('Navigate to: ' + str(x) + ' | ' + str(y))

        target_x = (x * self.map_info.resolution) + self.map_info.origin.position.x
        target_y = (y * self.map_info.resolution) + self.map_info.origin.position.y

        goal = Pose()
        goal.position.x = target_x
        goal.position.y = target_y
        goal.orientation.w = 1

        self.pub_goal.publish(goal)

    def _goal_reached_handle(self, reached):
        """
        Handles the notification that goal is reached
        """
        print('Reached: ' + str(reached))
        if reached:
            # check if position is the tag position
            if self.pose_converted.x >= self.approaching.x - 2 and self.pose_converted.x <= self.approaching.x + 2 and self.pose_converted.y >= self.approaching.y - 2 and self.pose_converted.y <= self.approaching.y + 1:
                bool_blob = Bool()
                bool_blob.data = False
                self.enable_blob_detection_service(bool_blob)
                bool_tag = Bool()
                bool_tag.data = False
                self.enable_tag_known_check_service(bool_tag)
                
                # publish that tag reached
                self.pub_coop_tag_reached.publish(self.approaching)
                count = 0
                while count < 100:
                    # stay at tag for 5 sec
                    count = count + 1
                    self.rate.sleep()
                print('--- tag reached ---')
                self.approaching = None
                self.waypoints = []
            self._navigate()
        else:
            self.waypoints = []
            self.is_navigating = False

if __name__ == "__main__":
    try:
        cts = CooperativeTagSearch()
        cts.run()
    except rospy.ROSInterruptException:
        pass