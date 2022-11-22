# V-REP Simulation from ELEC3210
## People
| Name         | Email                   | Task |
| ------------ | ----------------------- | ---- |
| Jiekai ZHANG | jzhanger@connect.ust.hk | 1, 2 |
## Before starting
1. Make sure your ROS interface of CoppeliaSim (or V-REP) is working fine
2. There are some modification to the ```env.ttt```, these are necessary for the project to run normally
   - Rename ```ROSInterface``` to ```ROS``` in the script of the bot
   - Changed the ```endPos``` in the script of the moving ball's path
3. Install the following ros package
   - joy
   - teleop_twist_keyboard
   - gmapping
## Tasks and method
### Task 1, Bulid a 2D map
#### Using hector_mapping
**Done**, but the performance is not quite well. It works only when the bot moves very slowly.  
But it do not require a odom frame, so very simple.  
Also a wired thing is the orientation of ```map``` frame and ```base_link``` is not aligned. I just added a frame to fix it but there should have a better way.
#### Using gmapping
Cannot get a very good odom using ```robot_localization``` so cannot use the map.  
May be try to integrate the odom by myself (should be much better by my test).  
P.S. There's no odom provide by the simulator and the control command ```/vrep/vel_cmd``` is not usable since the real speed in the simulator does NOT match the input speed. I really wonder why there is a such stupid mistake.
#### 
### Task 2, Use keyboard to control the bot
**Done**, using ```teleop_twist_keyboard``` for keyboard and ```joy``` for joy con.  
A unsatisfying fact is that the ```teleop_twist_keyboard``` will mess up the terminal output, looks very ugly. Maybe try ```turtlebot_teleop``` later. Currently I only use joy con for controlling. 
### Task 3, Image localization
### Task 4, Follow the sphere
### Task 5, Localizaiton
### Task 6, Launch file
### Bonus Task, Automatic exploration
### Bonus Task, Fusion of LiDAR and camera