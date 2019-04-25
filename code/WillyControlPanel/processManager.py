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
		self.rosProcess = subprocess.Popen("/home/willy/Documents/controlWilly/startroscore.sh", preexec_fn=os.setpgrp)
		# Create and start a thread for the ros vars monitor
		self.rmonThread = multiprocessing.Process(target=CWListener, args=(self.queue,))
		self.rmonThread.daemon = True
		self.rmonThread.start()
		print("ROS Core Process started. PID: " + str(self.rosProcess.pid))

	# Start the motor controller
	def startMC(self):
		self.motorProcess = subprocess.Popen("/home/willy/Documents/controlWilly/startmotorcon.sh", preexec_fn=os.setpgrp)
		print("Motor Controller Process started. PID: " + str(self.motorProcess.pid))

	# Start the joystick interface
	def startJoy(self):
		self.joyProcess = subprocess.Popen("/home/willy/Documents/controlWilly/startjoystick.sh", preexec_fn=os.setpgrp)
		print("Joystick Interface Process started. PID: " + str(self.joyProcess.pid))

	def stopProcess(self, process):
		self.process = process
		self.time_limit = 10

		sleep(self.time_limit)
		os.kill(self.process.pid, SIGTERM)