import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
import os
import random
from time import sleep

goalSounds = {
	"smb_goal": "~/Music/goal_reached/smb_goal.wav",
	"ff3_goal": "~/Music/goal_reached/ff3_victory.wav",
	"mm2_goal": "~/Music/goal_reached/mm2_victory.wav",
	"sor2_levelclear": "~/Music/goal_reached/sor2_levelclear.wav",
	"s3k_actclear": "~/Music/goal_reached/s3k_actclear.wav"
	}

class StatusChecker(object):
	def __init__(self):
		self.previous = None

	def callback(self, data):
		try:
			status = data.status_list[0].status
			if status != self.previous:
				if status == 1:
					print("New goal received")
				elif status == 3:
					print("Goal reached")
					playSnd(3)
				elif status == 4:
					print("Goal aborted.")
					playSnd(4)
				self.previous = status
		except:
			print("ROS isn't sending anything yet")
			sleep(5)
			pass
			
		

def playSnd(sound):
	if sound == 3:
		os.system("aplay "+ random.choice(goalSounds))
		return
	if sound == 4:
		os.system("aplay ~/Music/goal_forfeit/smb_playerdown.wav")
		return

checker = StatusChecker()
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("move_base/status", GoalStatusArray, checker.callback)
rospy.spin()

