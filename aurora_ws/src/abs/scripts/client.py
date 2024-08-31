import roslib
roslib.load_manifest('abs')
import rospy
import actionlib

from abs.msg import BewaessernAction, BewaessernGoal # type: ignore

if __name__ == 'main':
    rospy.init_node('Bewaesserungsnode')
    client = actionlib.SimpleActionClient('bewaessern', BewaessernAction)
    client.wait_for_server()

    goal = BewaessernGoal()

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))