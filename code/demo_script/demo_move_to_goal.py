#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

# Creating additional poses within pose_callback needs to be done by the following convention:
#Pose(Point(x, y, z), Quaternion(x, y, z, w)),
# Example, to move forward on the x axis, facing north:
#Pose(Point(1.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),

# Quaternion orientations (rounded):
# North: 0 0 0 1
# East: 0 0 -0.7 0.7
# South: 0 0 1 0
# West: 0 0 0.7 0.7

positions = []

# Callback that gets run once amcl_pose publishes the current position of the robot
def pose_callback(msg):
    global posesub
    print(msg)
    # Put coordinates in shorter variable, faster to call coordinates this way
    pose = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    # Debug: Print current pose and orientation
    #print("-------------- current position --------------")
    #print(pose)
    #print(orientation)

    # On configuring the intended movement plan, poses are calculated incrementally from a starting point. 
    #After assigning an estimated pose within RViz, the coordinates from this estimated pose are returned by AMCL

    # Form a new pose from AMCL's reported location of the robot on the map.
    new_pose = Pose(Point(pose.x + 3.0, pose.y, pose.z), Quaternion(orientation.x, orientation.y, orientation.z, orientation.w))
    # Debug: Ensure the coordinates have been added upon the existing ones.
    #print("-------------- new position     --------------")
    #print(new_pose)

    # Keep on forging new poses until the cube or plan is complete.
    # Each new pose adds up on the previous pose, ensuring that the robot will move up 1 meter, then right 1 meter from the new position, then down 1 meter, et cetera.
    npp1 = new_pose
    new_pose2 = Pose(Point(npp1.position.x, npp1.position.y - 5.0, npp1.position.z), Quaternion(npp1.orientation.x, npp1.orientation.y, npp1.orientation.z - 0.7, npp1.orientation.w - 0.3))
    npp2 = new_pose2
    new_pose3 = Pose(Point(npp2.position.x - 3.0, npp2.position.y, npp2.position.z), Quaternion(npp2.orientation.x, npp2.orientation.y, npp2.orientation.z + 1.7, npp2.orientation.w - 0.7))
    npp3 = new_pose3
    new_pose4 = Pose(Point(npp3.position.x, npp3.position.y + 5.0, npp3.position.z), Quaternion(npp3.orientation.x, npp3.orientation.y, npp3.orientation.z - 0.3, npp3.orientation.w + 0.7))
    # Once finished, extend the (empty!) array with the new poses.
    positions.extend([new_pose, new_pose2, new_pose3, new_pose4])
    # Unsubscribe from amcl_pose until the next time the script is called.
    return posesub.unregister()

def movebase_client(position):
    # Create actionlib client for move base to send goal and get feedback.
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    # Create goal message using required variables and intended goal position
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = position

    # Send the goal message to movebase and wait for the result
    print("Sending goal to move_base")
    client.send_goal(goal)

    wait = client.wait_for_result()
    # If there's no message received, assume move_base might be down or inaccessible.
    # Otherwise, return the result.
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print(client.get_result())
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        # Define subscriber, wait for message to be published, then run pose_callback
        global posesub
        posesub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        # After callback has finished and there are four positions in the array.
        if positions >= 3:
            # Iterate through each intended pose and run the movebase_client to send the goal.
            # Once the goal is reached, a result is returned if successful, in that case: continue the loop until all goals have been met.
            for target_pose in positions:
                result = movebase_client(target_pose)
                if result:
                    print("Goal reached. Moving to next one...")
                rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")