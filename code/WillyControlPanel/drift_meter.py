import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu

class DriftMeasure:
	def laser_cb(self, data):
		#print("-------- laser pose message      --------")
		#print(data)
		self.laser_position = data.pose.position
		self.laser_orientation = data.pose.orientation

		self.calculate_drift()

	def imu_cb(self, data):
		#print("-------- IMU orientation message --------")
		#print(data)
		self.imu_orientation = data.orientation

		self.calculate_drift()

	def amclpose_cb(self, data):
		#print("-------- AMCL pose message       --------")
		#print(data)
		self.amcl_pose = data.pose.pose.position
		self.amcl_orientation = data.pose.pose.orientation

		self.calculate_drift()

	def calculate_drift(self):
		# First the difference between IMU and laser orientation
		orientation_diff = Quaternion()
		orientation_diff.x = abs(self.laser_orientation.x - self.imu_orientation.x)
		orientation_diff.y = abs(self.laser_orientation.y - self.imu_orientation.y)
		orientation_diff.z = abs(self.laser_orientation.z - self.imu_orientation.z)
		orientation_diff.w = abs(self.laser_orientation.w - self.imu_orientation.w)
		print("-----------")
		print("laser ori.: X: " + str(self.laser_orientation.x) + " Y: " + str(self.laser_orientation.y) + " Z: " + str(self.laser_orientation.z) + " W: " + str(self.laser_orientation.w))
		print("IMU orien.: X: " + str(self.imu_orientation.x) + " Y: " + str(self.imu_orientation.y) + " Z: " + str(self.imu_orientation.z) + " W: " + str(self.imu_orientation.w))
		print("Difference: X: " + str(orientation_diff.x) + " Y: " + str(orientation_diff.y) + " Z: " + str(orientation_diff.z) + " W: " + str(orientation_diff.w))

	def __init__(self):
		rospy.Subscriber('/pose_stamped', PoseStamped, self.laser_cb)
		rospy.Subscriber('/imu/data', Imu, self.imu_cb)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amclpose_cb)

if __name__ == '__main__':
	try:
		rospy.init_node('drift_measurement_node')
		drift_measure = DriftMeasure()
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Drift measurement halted.")