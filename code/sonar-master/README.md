# Sonar

[![Build Status](https://travis-ci.org/Windesheim-Willy/sonar.svg?branch=master)](https://travis-ci.org/Windesheim-Willy/sonar)

The sonar is a node to process the raw sonar information to the sonar topic.

For more information, check out the wiki about the sonar [Wiki](https://windesheim-willy.github.io/WillyWiki/Components/sonar.html)

Install:

cd ~/catkin_workspace/src/
sudo git clone https://github.com/Windesheim-Willy/sonar.git sonar


sudo nano /etc/udev/rules.d/99-usb-serial.rules

KERNEL=="ttyACM*", ATTRS{idVendor}=="2a03", SYMLINK+="willy_sonar"

udevadm control --reload-rules
