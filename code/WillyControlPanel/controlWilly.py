import Tkinter as tk
import ttk
import sys
import os
import time
import threading

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
	# Create and start a thread for the process monitor
	procThread = threading.Thread(target=processMonitor.processMonitor(cwpanel))
	procThread.setDaemon(True)
	# Start the window's main loop
	root.mainloop()
	pingThread.start()
	procThread.start()


# Runs on exit(ctrl+c or window x button)
if __name__ == '__main__':
	sys.exit(main(sys.argv))
