import Tkinter as tk
import ttk

# Class for the control panel and its many subcomponents and specific functions.
class cwPanel():
	# On initialization, the whole window and its contents will be drawn.
	def __init__(self, parent, procman):
	###### WINDOW CREATION STARTS HERE ######
		# Define root window settings
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

		####### Top right - ping monitor to determine status of pi's and lidar
		#self.pingLabel = tk.Label(self.topright_frame, text="Components:")
		#self.pingLabel.grid(row=0, column=0, padx=3, pady=4, ipadx=0, ipady=0)
		## LIDAR sensor label
		#self.pingLidar = tk.Label(self.topright_frame, text="LIDAR")
		#self.pingLidar.grid(row=0, column=1, padx=5, pady=4, ipadx=5, ipady=0)
		## Sensor Pi label
		#self.pingSensorPi = tk.Label(self.topright_frame, text="Sensor Pi")
		#self.pingSensorPi.grid(row=0, column=2, padx=5, pady=4, ipadx=5, ipady=0)
		## Social Interaction Pi label
		#self.pingSiPi = tk.Label(self.topright_frame, text="SI Pi")
		#self.pingSiPi.grid(row=0, column=3, padx=5, pady=4, ipadx=5, ipady=0)
		## Brain Pi label
		#self.pingBrainPi = tk.Label(self.topright_frame, text="Brain Pi")
		#self.pingBrainPi.grid(row=0, column=4, padx=5, pady=4, ipadx=5, ipady=0)
		## Router Pi label
		#self.pingRouterPi = tk.Label(self.topright_frame, text="Router Pi")
		#self.pingRouterPi.grid(row=0, column=5, padx=5, pady=4, ipadx=5, ipady=0)

		###### Bottom left - per-service start/stop/reboot buttons
		# Initiate process manager for these
		# Then define frame and buttons for each process
		self.ROSbtnFrame = tk.LabelFrame(self.bottomleft_frame, text="ROS Master", relief="raised", width=190, height=40)
		self.ROSbtnFrame.place(x=0,y=0)
		self.startROSbtn = ttk.Button(self.ROSbtnFrame, text="Start", command=procman.startROS)
		self.startROSbtn.pack(side="left")
		self.stopROSbtn = ttk.Button(self.ROSbtnFrame, text="Stop", command=procman.stopROS)
		self.stopROSbtn.pack(side="left")

		self.MCbtnFrame = tk.LabelFrame(self.bottomleft_frame, text="Motor Controller", relief="raised", width=190, height=40)
		self.MCbtnFrame.place(x=0,y=50)
		self.startMCbtn = ttk.Button(self.MCbtnFrame, text="Start", command=procman.startMotor)
		self.startMCbtn.pack(side="left")
		self.stopMCbtn = ttk.Button(self.MCbtnFrame, text="Stop", command=procman.stopMotor)
		self.stopMCbtn.pack(side="left")

		self.JoybtnFrame = tk.LabelFrame(self.bottomleft_frame, text="Joystick interface", relief="raised", width=190, height=40)
		self.JoybtnFrame.place(x=0,y=100)
		self.startJoybtn = ttk.Button(self.JoybtnFrame, text="Start", command=procman.startJoy)
		self.startJoybtn.pack(side="left")
		self.stopJoybtn = ttk.Button(self.JoybtnFrame, text="Stop", command=procman.stopJoy)
		self.stopJoybtn.pack(side="left")

		###### Top left - Master switches
		self.MasterOnBtn = ttk.Button(self.topleft_frame, text="Start All", command=procman.startAll)
		self.MasterOnBtn.pack(side="left")
		self.MasterOffBtn = ttk.Button(self.topleft_frame, text="Stop All", command=procman.stopAll)
		self.MasterOffBtn.pack(side="left")

		###### Bottom right: ROSTopic outputs
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