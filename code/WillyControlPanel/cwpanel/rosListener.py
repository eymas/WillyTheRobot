import rospy
import rosgraph
import tf
import geometry_msgs.msg

class CWListener(object):
	def __init__(self):
		print("Starting ROS Listener")
		rospy.init_node('controlWilly_Listener')
		rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, self.velCallback)
		
	def velCallback(self, data):
		
		vel = data.linear.x
		rot = data.angular.z

		print(vel)
		print(rot)

		self.queue.put(vel)

# Callback for rostopic cmd_vel updates		
	def getQueue(self, queue):
		while True:
			try:
				pack = self.queue.get_nowait()
				if pack:
					print("Got pack")
					print(pack)
					self.velScale.set(pack.vel)
					self.rotScale.set(pack.rot)
				else:
					print("Did not get pack")
					pass
			except Queue.Empty:
				# Did not get pack, trying later.
				pass
			time.sleep(1)