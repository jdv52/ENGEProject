# Setting up ROS nodes on EV3 to communicate with Raspberry Pi
## Set up the Raspberry Pi
  - The Raspberry Pi runs ROS noetic, which is available for Ubuntu 20.04. Use [this tutorial](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview) to set that up. *Note: use ethernet connection at first and install a desktop. Desktop install is optional, but it makes it easier to connect to eduroam.*
  - Follow [this tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS Noetic on the Raspberry Pi.
  - Download the 'catkin_ws' folder on the Raspberry Pi. Navigate to the catkin_ws directory and run:
    ```
    catkin_make
    ```
  - While in the catkin_ws directory, also remember to source the work space by running:
    ```
    source devel/setup.bash
    ```
  - Raspberry Pi setup is now complete and you should now be able to run roslaunch file (after sourcing the workspace) included in the workspace:
    ```
    roslaunch navigation_pkg nav_stack.launch
    ```
## Set up EV3
  - Follow ev3dev's [Getting Started](https://www.ev3dev.org/docs/getting-started/) tutorial to flash the EV3 with the ev3dev image.
  - Follow [this tutorial](https://www.ev3dev.org/docs/tutorials/connecting-to-the-internet-via-usb/) to allow the EV3 to connect to wifi via USB tethering to the Raspberry Pi.
  - SSH into the EV3 by running
    ```
    ssh robot@ev3dev.local
    ```
  - Run:
    ```
    sudo apt udpate
    sudo apt upgrade
    sudo apt install unzip bzip2 build-essential
    ```
  - Install ROS system dependencies listed in step 2 of the "Install ros_comm" section of [this tutorial](https://github.com/moriarty/ros-ev3/blob/master/brickstrap-build-status.md)
  - Install bootstrap dependencies with:
    ```
    sudo pip install -U rosdep rosinstall_generator wstool rosinstall
    ```
  - Initialize rosdep:
    ```
    sudo rosdep init
    sudo rosdep update
    ```
  - Create a catkin workspace to build the core ROS packages:
    ```
    mkdir ~/ros_catkin_ws
    cd ~/ros_catkin_ws
    ```
  - Install ros_comm, common_msgs, and geometry_msgs:
    ```
    rosinstall_generator ros_comm common_msgs geometry_msgs --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
    wstool init src kinetic-ros_comm-wet.rosinstall
    ```
  - Check for/install all required system dependencies:
    ```
    rosdep install --from-paths src --ignore-src -r -y
    ```
  - Build the catkin workspace
    ```
    ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
    ```
  - Source the workspace
    ```
    source ~/ros_catkin_ws/install_isolated/setup.bash
    ```
## Running the Application
  - Make sure to source the appropriate workspaces on both the EV3 and the Raspberry Pi
