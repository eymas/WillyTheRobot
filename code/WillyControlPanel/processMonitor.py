import os
import subprocess
from threading import Thread
import time

# Monitoring Function to check each of the three processes their status
class processMonitor:
	def __init__(self, root, cwpanel):
		self.root = root
		self.window = cwpanel
		print("Process Monitor called")

	def run(self):
		# Define /dev/null to throw output towards lest the logs get cluttered with PIDs
		FNULL = open(os.devnull, 'w')
		# Try each command and read the output.
		# Each of those subprocesses will return 0 if successful.
		# if it is anything else than 0, an error occurred within the application it calls or the subprocess module itself.
		try:
			monROS = subprocess.call(["pgrep", "roscore"], stdout=FNULL)
			monMC = subprocess.call(["pgrep", "-f", "python /opt/ros/kinetic/lib/rosserial_python"], stdout=FNULL)
			monJoy = subprocess.call(["pgrep", "-f", "teleop_twist.joy"], stdout=FNULL)
		except subprocess.CalledProcessError as e:
			print("Caught an exception: "+ e)
			pass

		# If the status for the ROS Core returned 0, if anything else; it probably isn't running or something's wrong with it.
		if monROS == 0:
			self.window.ROSbtnFrame.config(background="green")
		else:
			self.window.ROSbtnFrame.config(background="red")

		# If the status for the Motor Controller returned 0, if anything else; it probably isn't running or something's wrong with it.
		if monMC == 0:
			self.window.MCbtnFrame.config(background="green")
		else:
			self.window.MCbtnFrame.config(background="red")

		# If the status for the Joystick Interface returned 0, if anything else; it probably isn't running or something's wrong with it.
		if monJoy == 0:
			self.window.JoybtnFrame.config(background="green")
		else:
			self.window.JoybtnFrame.config(background="red")
		self.root.after(5, self.run)