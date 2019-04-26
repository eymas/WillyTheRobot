import os
import subprocess
import time

# Monitoring function to ping for the four raspberry pi's and the LIDAR sensor.
def pingMonitor(cwpanel):
	print("[Ping] Pingmonitor is running")
	# Define /dev/null to throw output towards lest the logs are filled with ping results
	FNULL = open(os.devnull, 'w')
	try:
		# Ping the lidar, change label background on status
		lidarPing = subprocess.call(["ping", "-c", "1", "192.168.0.180"], stdout=FNULL, stderr=subprocess.STDOUT)
		cwpanel.pingLidar.config(background="green") if lidarPing == 0 else cwpanel.pingLidar.config(background="red")
	except CalledProcessError as e:
		print("Caught an Exception: "+ e)
		
	try:	
		# Ping the sensor pi, change label background on status
		sensorPing = subprocess.call(["ping", "-c", "1", "192.168.0.13"], stdout=FNULL, stderr=subprocess.STDOUT)
		cwpanel.pingSensorPi.config(background="green") if sensorPing == 0 else cwpanel.pingSensorPi.config(background="red")
	except CalledProcessError as e:
		print("Caught an Exception: "+ e)

	try:
		# Ping the router pi, change label background on status
		routerPing = subprocess.call(["ping", "-c", "1", "192.168.0.1"], stdout=FNULL, stderr=subprocess.STDOUT)
		cwpanel.pingRouterPi.config(background="green") if routerPing == 0 else cwpanel.pingRouterPi.config(background="red")
	except CalledProcessError as e:
		print("Caught an Exception: "+ e)

	try:
		# Ping the SI pi, change label background on status
		siPing = subprocess.call(["ping", "-c", "1", "192.168.0.11"], stdout=FNULL, stderr=subprocess.STDOUT)
		cwpanel.pingSiPi.config(background="green") if siPing == 0 else cwpanel.pingSiPi.config(background="red")
	except CalledProcessError as e:
		print("Caught an Exception: "+ e)

	try:
		# Ping the brain pi, change label background on status
		brainPing = subprocess.call(["ping", "-c", "1", "192.168.0.12"], stdout=FNULL, stderr=subprocess.STDOUT)
		cwpanel.pingBrainPi.config(background="green") if brainPing == 0 else cwpanel.pingBrainPi.config(background="red")
	except CalledProcessError as e:
		print("Caught an Exception: "+ e)