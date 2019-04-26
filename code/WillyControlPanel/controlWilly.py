from mttkinter import mtTkinter as tk
import ttk
import sys
import os
import time
import threading

import cwPanel
import processManager
from processMonitor import processMonitor
from pingMonitor import pingMonitor

####### MAIN WINDOW LOOP #######
# Activate window main loop.
def main(args):
	# Create and define the root window and controlpanel class
	root = tk.Tk()
	cwpanel = cwPanel.cwPanel(root)
	root.after(1000, pingMonitor, cwpanel)
	root.mainloop()
	# Create and start a thread for the ping monitor
	#processMonitor(cwpanel)
	# Start main window loop
	
		

# Runs on exit(ctrl+c or window x button)
if __name__ == '__main__':
	sys.exit(main(sys.argv))
