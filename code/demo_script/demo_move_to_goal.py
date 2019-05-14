#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

position1 = Pose(Point(0.7, 0.0, 0.0), Quaternion(0.0, 0.0, 0.99, 1.0))
position2 = Pose(Point(0.0, 0.7, 0.0), Quaternion(0.0, 0.0, 0.5, 0.0))
position2 = Pose(Point(-0.7, 0.0, 0.0), Quaternion(0.0, 0.0, -0.99, 0.0))
position2 = Pose(Point(0.0, -0.7, 0.0), Quaternion(0.0, 0.0, 0.5, 0.0))

def odom_callback(msg):
    print(msg.pose.pose)

def movebase_client(position):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = position

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rospy.Subscriber('odom', Odometry, odom_callback)
        result1 = movebase_client(position1)
        if result1:
            print("First goal reached.")
            result2 = movebase_client(position2)
        if result2:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")