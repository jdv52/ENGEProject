# Setting up ROS nodes on EV3 to communicate with Raspberry Pi
## Set up the Raspberry Pi
  - The Raspberry Pi runs ROS noetic, which is available for Ubuntu 20.04. Use [this tutorial](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview) to set that up. *Note: use ethernet connection at first and install a desktop. Desktop install is optional, but it makes it easier to connect to eduroam.*
  - Follow [this tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS Noetic on the Raspberry Pi.
  - Download the 'catkin_ws' folder to the Raspberry Pi and run:

    `catkin_make`
  
  - Raspberry Pi setup is now complete and you should now be able to run `roscore` and the roslaunch file included in the workspace (`roslaunch [launch file name]`).

## Set up EV3
  - Follow ev3dev's [Getting Started](https://www.ev3dev.org/docs/getting-started/) tutorial to prepare the EV3.
  - Follow [this tutorial](https://www.ev3dev.org/docs/tutorials/connecting-to-the-internet-via-usb/) to allow the EV3 to connect to wifi via USB tethering to the Raspberry Pi.
  - SSH into the EV3 and install ROS (exact packages that need to be installed are TBD)

## Running the Application
  - Navigate to the catkin_ws directory and run the provided shell script (Not yet available).
  - Execute the ev3_ctrl node on the EV3 brick.
