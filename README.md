# schunk_grippers
PG70 and EZN64 Xacro models and drivers for serial communication (RS232, USB). See [Wiki page](http://wiki.ros.org/schunk_grippers) for more details

## PG70 package
- uses [ROS serial library](http://wiki.ros.org/serial) package to interface pg70 controller over serial port
- provides **ROS services** for basic gripper control (get_position, set_position, reference, get_error, acknowledge_error, stop) 
- allows user to set goal position with velocity and acceleration
- includes **Xacro model** for easier integration with various robots
- automatically reads and broadcasts actual gripper position to **tf**

## EZN64 package
- uses [libusb1-0](http://www.libusb.org/wiki/libusb-1.0) library to interface ezn64 controller over USB port
- provides **ROS services** for basic gripper control (get_position, set_position, reference, get_error, acknowledge_error, stop)
- allows user to set only goal position, velocity and acceleration setpoints are not supported within current version
- includes **Xacro model** for easier integration with various robots
- automatically reads and broadcasts actual gripper position to **tf**

##Demonstration RVIZ
Schunk grippers + Motoman SDA10F in simulation

<a href="http://www.youtube.com/watch?feature=player_embedded&v=NLtPqIC4rdg
" target="_blank"><img src="http://img.youtube.com/vi/NLtPqIC4rdg/0.jpg" 
alt="Schunk grippers + Motoman SDA10F" width="480" height="360" border="10" /></a>

##Demonstration with real robot
<a href="http://www.youtube.com/watch?feature=player_embedded&v=8eH11rpk4X0
" target="_blank"><img src="http://img.youtube.com/vi/8eH11rpk4X0/0.jpg" 
alt="Schunk grippers + Motoman SDA10F" width="480" height="360" border="10" /></a>

## Configure the supervisor configuration file

**Note**: The following instructions are to be used as a template. Don't simply copy-paste stuff in the configuration file and expect it to work. Read carefuly and adjust according to your environment

To simplify your life, we included a script (`util/supervisor.sh`) that can be run from `supervisor`. This script takes care to set up the environment variables so that the running of the commands is smoother. Pay extra attention to the line that specifies the `ROS_MASTER_URI` environment variable and adjust it to your needs.

Open `supervisor`'s configuration file (`/etc/supervisor/supervisord.conf`) with your favorite editor (don't forget to `sudo`) and add the following lines at the bottom:
```
[program:schunk_pg70]
command=/bin/bash /home/<USER>/<ROS_WORKSPACE>/src/schunk_pg70/util/supervisor.sh
user=<USER>
autorestart=true
startretries=100
stdout_logfile=</path/to/log/file.log>
stderr_logfile=</path/to/errorlog/file.log>
```

Replace `<USER>` with the name of your user.

Run supervisor:
```
$ supervisor
```



