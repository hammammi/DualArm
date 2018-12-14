# DualArm
DualArmProject

Epos-library : for rasp. in desktop

for raspberry pi firefox

$ wget launchpad.net/~ubuntu-mozilla-security/+archive/ubuntu/ppa/+files/firefox_52.0.2+build1-0ubuntu0.12.04.1_armhf.deb

$ sudo apt-get purge firefox

$ sudo dpkg -i firefox_52.0.2+build1-0ubuntu0.12.04.1_armhf.deb

when /opt permission denied error occurs:

$ sudo chown -R $USER /opt
