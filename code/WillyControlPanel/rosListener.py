import rospy
import rosgraph
import tf
import geometry_msgs.msg

class CWListener(object):
	def __init__(self, queue):
		print("Starting ROS Listener")
		rospy.init_node('controlWilly_Listener')
		rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, self.velCallback)
		self.queue = queue
		
	def velCallback(self, data):
		
		vel = data.linear.x
		rot = data.angular.z

		print(vel)
		print(rot)

		self.queue.put(vel)