import os
import signal
import psutil
import subprocess
from time import sleep


# Class for ROS Core process control/monitoring
class processManager():
	def __init__(self):
		pass

	# Start every service in order.
	def startAll(self):
		print("Starting all services...")
		self.startROS()
		time.sleep(7)
		self.startMC()
		time.sleep(2)
		self.startJoy()

	# Stop every service in order
	def stopAll(self):
		print("Stopping all services...")
		stopProcess(self.rosProcess)
		stopProcess(self.motorProcess)
		stopProcess(self.joyProcess)

	# Start the ROS Core
	def startROS(self):
		self.rosProcess = subprocess.Popen(["/home/willy/Documents/controlWilly/scripts/startroscore.sh"], preexec_fn=os.setpgrp)
		print("ROS Core Process started. PID: " + str(self.rosProcess.pid))

	def stopROS(self):
		rp_pid = self.rosProcess.pid
		print(jp_pid)
		print(type(jp_pid))
		self.kill_proc_tree(rp_pid)

	# Start the motor controller
	def startMotor(self):
		self.motorProcess = subprocess.Popen(["/home/willy/Documents/controlWilly/scripts/startmotorcon.sh"], preexec_fn=os.setpgrp)
		print("Motor Controller Process started. PID: " + str(self.motorProcess.pid))

	def stopMotor(self):
		mc_pid = self.motorProcess.pid
		print(jp_pid)
		print(type(jp_pid))
		self.kill_proc_tree(mc_pid)

	# Start the joystick interface
	def startJoy(self):
		self.joyProcess = subprocess.Popen(["/home/willy/Documents/controlWilly/scripts/startjoystick.sh"], preexec_fn=os.setpgrp)
		print("Joystick Interface Process started. PID: " + str(self.joyProcess.pid))

	def stopJoy(self):
		jp_pid = self.joyProcess.pid
		print(jp_pid)
		print(type(jp_pid))
		self.kill_proc_tree(jp_pid)

	def kill_proc_tree(self, pid, including_parent=True):    
		parent = psutil.Process(pid)
		children = parent.children(recursive=True)
		for child in children:
			child.kill()
		gone, still_alive = psutil.wait_procs(children, timeout=5)
		if including_parent:
			parent.kill()
			parent.wait(5)
