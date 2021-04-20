#BOBBY THE DRONE
![alt text](../img/bobby_the_drone.png "Bobby, our beloved drone <3")

## Inspection drone project at ENPC (2020-2021)

This repository holds the code associated with the inspection drone project. This department project is aimed at
creating an autonomous drone for building inspection. 

We focus here on obstacle detection during a waypoint mission. 
The associated Google Drive can be found here : https://drive.google.com/drive/folders/1pWQdOV-hi2a7xwepPXjL5-DVYdoxBjQJ?usp=sharing. It is where we store the advancement of the project. 

Our drone is based on Pixhawk4 Mini architecture with a DJI F450 frame running ardupilot : https://ardupilot.org/copter/

We use dronekit as our main mavlink environment : http://dronekit.io/

For the moment we are in a testing phase, the test-scripts folders holds all the required scripts that test if your system is well functionning or not

##How to install and use

We run all of our scripts on a raspberry pi based on Raspberry PI OS (see : https://www.raspberrypi.org/documentation/)

To avoid having to use an external monitor everytime you use the Raspberry Pi, we recommend using SSH and VNC to control the raspberry pi : 

You will need to install on this device : dronekit, pymavlink. Those run on python 2.7 so use ``` sudo pip install ``` instead of ``` sudo pip3 install ```

Our development platform use PyCharm Pro as the primary IDE for Python, because it allows for quick deployement of code on the raspberry pi using ssh. See : https://www.jetbrains.com/help/pycharm/running-ssh-terminal.html for more instructions. Although SSH deployement is a Pro feature, Pycharm Pro is available for free for ENPC students. Otherwise you can implement ssh deployment using a small script (that you'll have to write... Have Fun !)

When testing the code in real condition (i.e. when the drone is flying) we recommend the user to generate a wifi network with the Raspberry Pi for easy troubleshooting and launching code using SSH. See : https://www.raspberrypi.org/documentation/configuration/wireless/access-point-routed.md and https://doc.ubuntu-fr.org/hostapd The range is quite good (10-15m).
You should also follow the [Test en vol checklist](../checklist/flying-test-checklist) (French only sorry).

To let the PixHawk4 Mini and the Raspberry Pi talk between each other, check this page : ( https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html ), and then follow the [Connection Checklist](../checklists/-test-checklist) available on this repository. In order for this to work well you'll need to provide good power supply to the Raspberry Pi, otherwise the Pi will slow down its internal clock, hence slowing its internal baudrate.


By Louis Reine, Thibaud Cambronne, supervized by Thomas Demmer.