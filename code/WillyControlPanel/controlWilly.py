import Tkinter as tk
import ttk
import sys
import os
import time
import subprocess
import threading
import multiprocessing
import Queue
import rospy
import rosgraph
import geometry_msgs.msg

# Class for the control panel and its many subcomponents and specific functions.
class cwPanel():
	# On initialization, the whole window and its contents will be drawn.
	def __init__(self, parent, queue):
	###### WINDOW CREATION STARTS HERE ######
		# Define root window settings
		self.queue = queue
		parent.geometry("645x355")
		parent.title("Willy the Robot - Control Panel")

		# Create the main frame
		self.mainframe = ttk.Frame(parent, padding="2 2 12 12")
		self.mainframe.pack(fill="both", expand=1)

		# Create the sub frames
		self.topleft_frame = ttk.LabelFrame(self.mainframe, padding="5 5 0 0", relief="sunken", text="Master Switches", width=205, height=50)
		self.topleft_frame.place(x=0, y=0)
		self.topright_frame = ttk.LabelFrame(self.mainframe, padding="5 5 0 0", relief="sunken", text="Ping monitor", width=435, height=50)
		self.topright_frame.place(x=205, y=0)
		self.bottomleft_frame = ttk.LabelFrame(self.mainframe, padding="5 5 0 0", relief="sunken", text="Services", width=205, height=300)
		self.bottomleft_frame.place(x=0, y=50)
		self.bottomright_frame = ttk.LabelFrame(self.mainframe, padding="5 5 0 0", relief="sunken", text="ROS Topics and Info", width=435, height=300)
		self.bottomright_frame.place(x=205, y=50)
		# Define widgets per frame
		# Top right - ping monitor to determine status of pi's and lidar
		self.pingLabel = tk.Label(self.topright_frame, text="Components:")
		self.pingLabel.grid(row=0, column=0, padx=3, pady=4, ipadx=0, ipady=0)
		# LIDAR sensor label
		self.pingLidar = tk.Label(self.topright_frame, text="LIDAR")
		self.pingLidar.grid(row=0, column=1, padx=5, pady=4, ipadx=5, ipady=0)
		# Sensor Pi label
		self.pingSensorPi = tk.Label(self.topright_frame, text="Sensor Pi")
		self.pingSensorPi.grid(row=0, column=2, padx=5, pady=4, ipadx=5, ipady=0)
		# Social Interaction Pi label
		self.pingSiPi = tk.Label(self.topright_frame, text="SI Pi")
		self.pingSiPi.grid(row=0, column=3, padx=5, pady=4, ipadx=5, ipady=0)
		# Brain Pi label
		self.pingBrainPi = tk.Label(self.topright_frame, text="Brain Pi")
		self.pingBrainPi.grid(row=0, column=4, padx=5, pady=4, ipadx=5, ipady=0)
		# Router Pi label
		self.pingRouterPi = tk.Label(self.topright_frame, text="Router Pi")
		self.pingRouterPi.grid(row=0, column=5, padx=5, pady=4, ipadx=5, ipady=0)

		# Bottom left - per-service start/stop/reboot buttons
		PP = ProcProc(self.queue)
		self.ROSbtnFrame = tk.LabelFrame(self.bottomleft_frame, text="ROS Master", relief="raised", width=190, height=40)
		self.ROSbtnFrame.place(x=0,y=0)
		self.startROSbtn = ttk.Button(self.ROSbtnFrame, text="Start", command=PP.startROS)
		self.startROSbtn.pack(side="left")
		self.stopROSbtn = ttk.Button(self.ROSbtnFrame, text="Stop", command=PP.stopROS)
		self.stopROSbtn.pack(side="left")

		self.MCbtnFrame = tk.LabelFrame(self.bottomleft_frame, text="Motor Controller", relief="raised", width=190, height=40)
		self.MCbtnFrame.place(x=0,y=50)
		self.startMCbtn = ttk.Button(self.MCbtnFrame, text="Start", command=PP.startMC)
		self.startMCbtn.pack(side="left")
		self.stopMCbtn = ttk.Button(self.MCbtnFrame, text="Stop", command=PP.stopMC)
		self.stopMCbtn.pack(side="left")

		self.JoybtnFrame = tk.LabelFrame(self.bottomleft_frame, text="Joystick interface", relief="raised", width=190, height=40)
		self.JoybtnFrame.place(x=0,y=100)
		self.startJoybtn = ttk.Button(self.JoybtnFrame, text="Start", command=PP.startJoy)
		self.startJoybtn.pack(side="left")
		self.stopJoybtn = ttk.Button(self.JoybtnFrame, text="Stop", command=PP.stopJoy)
		self.stopJoybtn.pack(side="left")

		# Top left - Master switches
		self.MasterOnBtn = ttk.Button(self.topleft_frame, text="Start All", command=PP.startAll)
		self.MasterOnBtn.pack(side="left")
		self.MasterOffBtn = ttk.Button(self.topleft_frame, text="Stop All", command=PP.stopAll)
		self.MasterOffBtn.pack(side="left")

		# Bottom right: ROSTopic outputs
		# Velocity speed indicator
		self.velScale = tk.Scale(self.bottomright_frame, label="Velocity", relief="raised", from_=0.7, to=-0.7, length=200, resolution=0.1, orient="vertical", font=("Helvetica", 24), fg="red")
		self.velScale.place(x=5, y=0)

		# Rotation speed indicator
		self.rotScale = tk.Scale(self.bottomright_frame, label="Rotation", relief="raised", from_=0.4, to=-0.4, length=142, resolution=0.1, orient="horizontal", font=("Helvetica", 24), fg="blue")
		self.rotScale.place(x=83, y=95)

		# Propagate packs to prevent frames from losing sizes
		parent.pack_propagate(0)
		self.mainframe.pack_propagate(0)
		self.topleft_frame.pack_propagate(0)
		self.topright_frame.pack_propagate(0)
		self.bottomleft_frame.pack_propagate(0)
		self.bottomright_frame.pack_propagate(0)
		self.ROSbtnFrame.pack_propagate(0)
		self.MCbtnFrame.pack_propagate(0)
		self.JoybtnFrame.pack_propagate(0)

	###### WINDOW CREATION ENDS HERE ######
	# From here we can define functions 

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
			time.sleep(5)
		

	# Monitoring Function to check each of the three processes their status
	def procMon(self):
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
			except CalledProcessError as e:
				print("Caught an exception: "+ e)

			# If the status for the ROS Core returned 0, if anything else; it probably isn't running or something's wrong with it.
			if monROS == 0:
				self.ROSbtnFrame.config(background="green")
			else:
				self.ROSbtnFrame.config(background="red")

			# If the status for the Motor Controller returned 0, if anything else; it probably isn't running or something's wrong with it.
			if monMC == 0:
				self.MCbtnFrame.config(background="green")
			else:
				self.MCbtnFrame.config(background="red")

			# If the status for the Joystick Interface returned 0, if anything else; it probably isn't running or something's wrong with it.
			if monJoy == 0:
				self.JoybtnFrame.config(background="green")
			else:
				self.JoybtnFrame.config(background="red")
			time.sleep(5)

	# Monitoring function to ping for the four raspberry pi's and the LIDAR sensor.
	def pingMon(self):
		# Define /dev/null to throw output towards lest the logs are filled with ping results
		FNULL = open(os.devnull, 'w')
		while True:
			try:
				# Ping the lidar, change label background on status
				lidarPing = subprocess.call(["ping", "-c", "1", "192.168.0.180"], stdout=FNULL, stderr=subprocess.STDOUT)
				self.pingLidar.config(background="green") if lidarPing == 0 else self.pingLidar.config(background="red")
			except CalledProcessError as e:
				print("Caught an Exception: "+ e)
				
			try:	
				# Ping the sensor pi, change label background on status
				sensorPing = subprocess.call(["ping", "-c", "1", "192.168.0.13"], stdout=FNULL, stderr=subprocess.STDOUT)
				self.pingSensorPi.config(background="green") if sensorPing == 0 else self.pingSensorPi.config(background="red")
			except CalledProcessError as e:
				print("Caught an Exception: "+ e)

			try:
				# Ping the router pi, change label background on status
				routerPing = subprocess.call(["ping", "-c", "1", "192.168.0.1"], stdout=FNULL, stderr=subprocess.STDOUT)
				self.pingRouterPi.config(background="green") if routerPing == 0 else self.pingRouterPi.config(background="red")
			except CalledProcessError as e:
				print("Caught an Exception: "+ e)

			try:
				# Ping the SI pi, change label background on status
				siPing = subprocess.call(["ping", "-c", "1", "192.168.0.11"], stdout=FNULL, stderr=subprocess.STDOUT)
				self.pingSiPi.config(background="green") if siPing == 0 else self.pingSiPi.config(background="red")
			except CalledProcessError as e:
				print("Caught an Exception: "+ e)

			try:
				# Ping the brain pi, change label background on status
				brainPing = subprocess.call(["ping", "-c", "1", "192.168.0.12"], stdout=FNULL, stderr=subprocess.STDOUT)
				self.pingBrainPi.config(background="green") if brainPing == 0 else self.pingBrainPi.config(background="red")
			except CalledProcessError as e:
				print("Caught an Exception: "+ e)
			# Wait 5 seconds between each polling session
			time.sleep(5)

####### EVERYTHING OUTSIDE OF THE MAIN GUI LOOP #######

class CWListener(object):
	def __init__(self, queue):
		self.queue = queue
		try:
			print("Trying to connect to ROS...")
			rospy.init_node('controlWilly_Listener')
		except Exception as e:
			print("Caught exception: " + e)
		if rosgraph.is_master_online():
			print("ROS Listener online.")
			while True:
				rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, self.velCallback)
				time.sleep(1)

	def velCallback(self, data):
		vel = data.linear.x
		rot = data.angular.z
		pack = (vel, rot)
		self.queue.put(pack)
		

# Class for ROS Core process control/monitoring
class ProcProc(object):
	def __init__(self, queue):
		self.queue = queue
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
		self.stopROS()
		self.stopMC()
		self.stopJoy()

	# Start the ROS Core
	def startROS(self):
		self.rosProcess = subprocess.Popen(["xterm", "-e", "/home/willy/Documents/controlWilly/startroscore.sh"])
		# Create and start a thread for the ros vars monitor
		self.rmonThread = multiprocessing.Process(target=CWListener, args=(self.queue,))
		self.rmonThread.daemon = True
		self.rmonThread.start()
		print("ROS Core Process started.")

	# Stop the ROS Core
	def stopROS(self):
		self.rosProcess.kill()
		self.rmonThread.kill()
		print("ROS Core Process terminated.")

	# Start the motor controller
	def startMC(self):
		self.motorProcess = subprocess.Popen(["xterm", "-e", "/home/willy/Documents/controlWilly/startmotorcon.sh"])
		print("Motor Controller Process started.")

	# Stop the motor controller
	def stopMC(self):
		self.motorProcess.kill()
		print("Motor Controller Process terminated.")

	# Start the joystick interface
	def startJoy(self):
		self.joyProcess = subprocess.Popen(["xterm", "-e", "/home/willy/Documents/controlWilly/startjoystick.sh"])
		print("Joystick Interface Process started.")

	# Stop the joystick interface
	def stopJoy(self):
		self.joyProcess.kill()
		print("Joystick Interface Process terminated.")

# Activate window main loop.
def main(args):
	# Create and define the root window and controlpanel class
	queue = Queue.Queue()
	root = tk.Tk()
	cwpanel = cwPanel(root, queue)
	# Create and start a thread for the ping monitor
	pingThread = threading.Thread(target=cwpanel.pingMon)
	pingThread.setDaemon(True)
	pingThread.start()
	# Create and start a thread for the process monitor
	procThread = threading.Thread(target=cwpanel.procMon)
	procThread.setDaemon(True)
	procThread.start()
	# Create and start a thread for the queue listener
	queueThread = threading.Thread(target=cwpanel.getQueue, args=(queue,))
	queueThread.setDaemon(True)
	queueThread.start()
	# Start the window's main loop
	root.mainloop()

# Runs on exit(ctrl+c or window x button)
if __name__ == '__main__':
	sys.exit(main(sys.argv))
