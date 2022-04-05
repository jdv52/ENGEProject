# ENGEProject

This code was written for a snowplow robot developed for the ENGE1216 class project.

This application takes advantage of ev3dev to run a ROS subscriber and publisher node on the Lego Mindstorms EV3 brick. A Raspberry Pi connected to the EV3 via USB runs roscore and provides higher level control of the robot (i.e. running encoder odometry and navigation planning).

# Project Status

Project is currently under development.

# Instructions

Read 'setup.md' for information on how to set up the Raspberry Pi and EV3 to run this project.

After setting up the Raspberry Pi and EV3, execute the ev3_ctrl node on the EV3 and run roscore on the Raspberry Pi.
