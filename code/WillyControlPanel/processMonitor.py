import os
import subprocess
import time

# Monitoring Function to check each of the three processes their status
class processMonitor():
	def __init__(self, cwpanel):
		# Define /dev/null to throw output towards lest the logs get cluttered with PIDs
		FNULL = open(os.devnull, 'w')
		while True:
			# Try each command and read the output.
			# Each of those subprocesses will return 0 if successful.
			# if it is anything else than 0, an error occurred within the application it calls or the subprocess module itself.
			try:
				monROS = subprocess.call(["pgrep", "roscore"], stdout=FNULL, stderr=subprocess.STDOUT)
				monMC = subprocess.call(["pgrep", "-f", "python /opt/ros/kinetic/lib/rosserial_python"], stdout=FNULL, stderr=subprocess.STDOUT)
				monJoy = subprocess.call(["pgrep", "-f", "teleop_twist.joy"], stdout=FNULL, stderr=subprocess.STDOUT)
			except subprocess.CalledProcessError as e:
				print("Caught an exception: "+ e)
				pass

			# If the status for the ROS Core returned 0, if anything else; it probably isn't running or something's wrong with it.
			if monROS == 0:
				cwpanel.ROSbtnFrame.config(background="green")
			else:
				cwpanel.ROSbtnFrame.config(background="red")

			# If the status for the Motor Controller returned 0, if anything else; it probably isn't running or something's wrong with it.
			if monMC == 0:
				cwpanel.MCbtnFrame.config(background="green")
			else:
				cwpanel.MCbtnFrame.config(background="red")

			# If the status for the Joystick Interface returned 0, if anything else; it probably isn't running or something's wrong with it.
			if monJoy == 0:
				cwpanel.JoybtnFrame.config(background="green")
			else:
				cwpanel.JoybtnFrame.config(background="red")
			time.sleep(1)