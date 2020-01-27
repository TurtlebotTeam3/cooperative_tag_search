import rospy

from path_planing.srv import FindPathToGoal
from path_planing.msg import FullPath, PathPoint

class CooperativeTagSearch:

    def __init__(self):
        rospy.init_node('cooperative_tag_search', anonymous=True)
        
        # --- Publishers ---

        # --- Subscribers ---

        # --- Service wait ---

        rospy.wait_for_service('find_path_to_goal')

        # --- Services ---
        self.find_path_to = rospy.ServiceProxy('find_path_to_goal', FindPathToGoal)

if __name__ == "__main__":
    try:
        cts = CooperativeTagSearch()
    except rospy.ROSInterruptException:
        pass