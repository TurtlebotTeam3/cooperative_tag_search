import rospy
import math
import tf

from path_planing.srv import FindPathToGoal, FindShortestPath
from path_planing.msg import FullPath, PathPoint

from tag_manager.srv import GetTags
from tag_manager.msg import TagList, TagPoint

from robot_localisation.srv import Localise

from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Bool

from nav_msgs.msg._OccupancyGrid import OccupancyGrid



class CooperativeTagSearch:

    def __init__(self):
        rospy.init_node('cooperative_tag_search', anonymous=True)
        
        self.localised = False
        self.tag_list = []
        self.tag_list_searching = []
        self.tag_list_found = []

        self.waypoints = []

        self.robot_pose_available = False
        self.robot_x = 0
        self.robot_x_old = 0
        self.robot_x_pose = 0
        self.robot_y = 0
        self.robot_y_old = 0
        self.robot_y_pose = 0
        self.robot_yaw = 0
        self.robot_yaw_old = 0
        self.pose = Pose()

        self.blowUpCellNum = 3
        self.robot_radius = 1

        self.map_resolution = 0
        self.map_offset_x = 0
        self.map_offset_y = 0

        self.is_navigating = False
        self.waypointsAvailable = False
        
        print("--- publisher ---")
        # --- Publishers ---
        self.pub_coop_tag_searching = rospy.Publisher('/coop_tag/searching', Point,queue_size=1)
        self.pub_coop_tag_reached = rospy.Publisher('/coop_tag/reached', Point,queue_size=1)
        self.pub_goal = rospy.Publisher('/move_to_goal/goal', Pose, queue_size=1)

        print("--- subscriber ---")
        # --- Subscribers ---
        self.sub_coop_tag_searching = rospy.Subscriber('/coop_tag/searching', Point, self._coop_tag_searching_receive)
        self.sub_coop_tag_reached = rospy.Subscriber('/coop_tag/reached', Point, self._coop_tag_reached_receive)
        self.pose_subscriber = rospy.Subscriber('/simple_odom_pose',Pose, self._update_pose)
        self.sub_goal_reached = rospy.Subscriber('/move_to_goal/reached', Bool, self._goal_reached_handle)
        
        print("--- service wait ---")
        # --- Service wait ---
        print("1")
        rospy.wait_for_service('find_shortest_path_service')
        print("2")
        rospy.wait_for_service('get_tags')
        print("3")
        rospy.wait_for_service('localise_robot_service')

        print("--- services ---")
        # --- Services ---
        self.find_shortest_path_service = rospy.ServiceProxy('find_shortest_path_service', FindShortestPath)
        self.get_tags_service = rospy.ServiceProxy('get_tags', GetTags)
        self.robot_localisation_service = rospy.ServiceProxy('localise_robot_service', Localise)

        print("--- setup ---")
        self._setup()
        print("--- ready ---")

    def _setup(self):
        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.map_resolution = map.info.resolution
        self.map_offset_x = map.info.origin.position.x
        self.map_offset_y = map.info.origin.position.y

    def _coop_tag_searching_receive(self, data):
        """
        Handles messages on topic /coop_tag/searching
        """
        self.tag_list.remove((int(data.Point.y, data.Point.x)))
        self.tag_list_searching.append((int(data.Point.y, data.Point.x)))

    def _coop_tag_reached_receive(self, data):
        """
        Handles messages on topic /coop_tag/reached
        """
        self.tag_list_searching.remove((int(data.Point.y, data.Point.x)))
        self.tag_list_found.remove((int(data.Point.y, data.Point.x)))

    def _update_pose(self, data):
        """
        Update Pose
        """
        if self.map_resolution > 0:
            self.pose.position.x = data.position.x
            self.pose.position.y = data.position.y
            self.pose.position.z = data.position.z
            self.pose.orientation.x = data.orientation.x 
            self.pose.orientation.y = data.orientation.y 
            self.pose.orientation.z = data.orientation.z
            self.pose.orientation.w = data.orientation.w 

            self.robot_y_pose = self.pose.position.y
            self.robot_x_pose = self.pose.position.x

            self.robot_x_old = self.robot_x
            self.robot_y_old = self.robot_y
            self.robot_yaw_old = self.robot_yaw

            self.robot_x = int(math.floor((self.robot_x_pose - self.map_offset_x)/self.map_resolution))
            self.robot_y = int(math.floor((self.robot_y_pose - self.map_offset_y)/self.map_resolution))
            self.robot_yaw = self._robot_angle()
            
            self.robot_pose_available = True

    def _robot_angle(self):
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def run(self):
        while not rospy.is_shutdown():
            #get the tag list
            if len(self.tag_list) == 0:
                service_response = self.get_tags_service()
                self._process_tag_list_from_service(service_response)

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
                                print('no more tags')


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
        service_response = self.find_shortest_path_service(self.blowUpCellNum, self.robot_x, self.robot_y, self.robot_radius, goals)
        
        # extract data
        waypoints = service_response.waypoints.fullpath
        allpoints = service_response.allPoints.fullpath

        return waypoints, allpoints, waypoints[-1]

    def _calculate(self):
        # select tag
        self.waypoints, _ , tag_to_approach = self._find_nearest_tag()
        
        # send to topic
        appraching = Point
        appraching.x = tag_to_approach.path_x
        appraching.y = tag_to_approach.path_y
        
        self.waypointsAvailable = True

    def _navigate(self):
        """
        Navigating to the waypoints
        """
        # get waypoint and start moving towards it
        # when success the process next
        self.is_navigating = True

        if(len(self.waypoints) > 0):
            print(self.waypoints)
            point = self.waypoints.pop(0)
            x = point.path_x
            y = point.path_y

            print(self.waypoints)

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
        goal = Pose()

        target_x = (x * self.map_resolution) + self.map_offset_x
        target_y = (y * self.map_resolution) + self.map_offset_y

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