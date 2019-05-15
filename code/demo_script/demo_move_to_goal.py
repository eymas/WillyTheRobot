#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

#Pose(Point(x, y, z), Quaternion(x, y, z, w)),
#Pose(Point(1.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.99, 1.0)),

# Quaternion orientations (rounded):
# North: 0 0 0 1
# East: 0 0 -0.7 0.7
# South: 0 0 1 0
# West: 0 0 0.7 0.7

positions = []

# Callback that gets run once amcl_pose publishes the current position of the robot
def pose_callback(msg):
    print(msg)
    # Put coordinates in shorter variable, faster to call coordinates this way
    pose = msg.position
    orientation = msg.orientation
    print("-------------- current position --------------")
    print(pose)
    print(orientation)
    # Form new pose.
    new_pose = Pose(Point(pose.x + 1.0, pose.y, pose.z), Quaternion(orientation.x, orientation.y, orientation.z, orientation.w))
    print("-------------- new position     --------------")
    print(new_pose)
    # Keep on forging new poses until the cube's complete.
    npp1 = new_pose
    new_pose2 = Pose(Point(npp1.position.x, npp1.position.y + 1.0, npp1.position.z), Quaternion(npp1.orientation.x, npp1.orientation.y, npp1.orientation.z - 0.7, npp1.orientation.w - 0.3))
    npp2 = new_pose2
    new_pose3 = Pose(Point(npp2.position.x - 1.0, npp2.position.y, npp2.position.z), Quaternion(npp2.orientation.x, npp2.orientation.y, npp2.orientation.z + 1.7, npp2.orientation.w - 0.7))
    npp3 = new_pose3
    new_pose4 = Pose(Point(npp3.position.x, npp3.position.y - 1.0, npp3.position.z), Quaternion(npp3.orientation.x, npp3.orientation.y, npp3.orientation.z - 0.3, npp3.orientation.w + 0.7))
    # Then throw them in an array for the movebase_client to process
    positions.extend([new_pose, new_pose2, new_pose3, new_pose4])

def movebase_client(position):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = position

    print("Sending goal to move_base")

    client.send_goal(goal)

    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print(client.get_result())
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rospy.Subscriber('/pose_stamped', PoseStamped, pose_callback)
        rospy.wait_for_message('/pose_stamped', PoseStamped)
        if positions >= 3:
            print("got here")
            print(positions)
            for target_pose in positions:
                result = movebase_client(target_pose)
                if result:
                    print("Goal reached. Moving to next one...")
                rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")