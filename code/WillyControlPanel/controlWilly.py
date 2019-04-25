import Tkinter as tk
import ttk
import sys
import os
import time
import threading

sys.path.insert(0, './cwpanel')
import cwPanel
import processManager
import processMonitor
import pingMonitor

####### MAIN WINDOW LOOP #######
# Activate window main loop.
def main(args):
	# Create and define the root window and controlpanel class
	root = tk.Tk()
	cwpanel = cwPanel.cwPanel(root)
	# Create and start a thread for the ping monitor
	pingThread = threading.Thread(target=pingMonitor.pingMonitor(cwpanel))
	pingThread.setDaemon(True)
	pingThread.start()
	# Create and start a thread for the process monitor
	procThread = threading.Thread(target=processMonitor.processMonitor(cwpanel))
	procThread.setDaemon(True)
	procThread.start()
	# Start the window's main loop
	root.mainloop()
	procMan = processManager.processManager()

# Runs on exit(ctrl+c or window x button)
if __name__ == '__main__':
	sys.exit(main(sys.argv))
