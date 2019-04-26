import os
import subprocess
import time

# Monitoring function to ping for the four raspberry pi's and the LIDAR sensor.
class pingMonitor:
	def __init__(self, root, cwpanel):
		self.root = root
		self.window = cwpanel
		print("[Ping] Pingmonitor is running")

	def run(self):
		# Define /dev/null to throw output towards lest the logs are filled with ping results
		DEVNULL = open(os.devnull, 'w')
		try:
			# Ping the lidar, change label background on status
			lidarPing = subprocess.call(["ping", "-w 1," "-c 1", "192.168.0.180"], stdout=DEVNULL)
		except subprocess.CalledProcessError as e:
			print("Caught an Exception: "+ e)
		self.window.pingLidar.config(background="green") if lidarPing == 0 else self.window.pingLidar.config(background="red")
			
		try:	
			# Ping the sensor pi, change label background on status
			sensorPing = subprocess.call(["ping", "-w 1," "-c 1", "192.168.0.13"], stdout=DEVNULL)
		except subprocess.CalledProcessError as e:
			print("Caught an Exception: "+ e)
		self.window.pingSensorPi.config(background="green") if sensorPing == 0 else self.window.pingSensorPi.config(background="red")

		try:
			# Ping the router pi, change label background on status
			routerPing = subprocess.call(["ping", "-w 1," "-c 1", "192.168.0.1"], stdout=DEVNULL)
		except subprocess.CalledProcessError as e:
			print("Caught an Exception: "+ e)
		self.window.pingRouterPi.config(background="green") if routerPing == 0 else self.window.pingRouterPi.config(background="red")

		try:
			# Ping the SI pi, change label background on status
			siPing = subprocess.call(["ping", "-w 1," "-c 1", "192.168.0.11"], stdout=DEVNULL)
		except subprocess.CalledProcessError as e:
			print("Caught an Exception: "+ e)
		self.window.pingSiPi.config(background="green") if siPing == 0 else self.window.pingSiPi.config(background="red")

		try:
			# Ping the brain pi, change label background on status
			brainPing = subprocess.call(["ping", "-w 1," "-c 1", "192.168.0.12"], stdout=DEVNULL)
		except subprocess.CalledProcessError as e:
			print("Caught an Exception: "+ e)
		self.window.pingBrainPi.config(background="green") if brainPing == 0 else self.window.pingBrainPi.config(background="red")

		print("Pinging")
		self.root.after(5, self.run)