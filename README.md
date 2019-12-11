# Corridor follow app for Parrot Ardrone 2.0

Here are two projects for a drone that should fly along a hallway using just the camera. The Vanishing Point app is good, but very sensitive to light changes. The second application is based on a neural network, being more robust, but having troubles detecting the end of the hallway. 

**References:**

**1.** [Dorbala,  V.S.,  Hafez,  A.H.A.,  and  Jawahar,  C.V.  (2019).A deep  learning  approach  for  robust  corridor  following  from an  arbitrary  pose. In 2019 27th Signal Processing and Communications Applications Conference (SIU), 1–4](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8806271)

**2.** [Előd Páll, Levente Tamás, Lucian Buşoniu, "Vision-Based Quadcopter Navigation in Structured Environments", In Handling Uncertainty and Networked Structure in Robot Control, Springer, Studies in Systems, Decision and Control Series, L. Busoniu, L. Tamas (editors), pp 265-290, 2016](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&cad=rja&uact=8&ved=2ahUKEwjd1IDvw-flAhXIhqQKHal8CasQFjABegQIABAC&url=http%3A%2F%2Fbusoniu.net%2Ffiles%2Fpapers%2Fquadchapter.pdf&usg=AOvVaw2CdSTWLK6VZ0GeUsUwLhk6)

## CNN App

In order to run the app you need to:
- download dataset (and the model): https://drive.google.com/open?id=11vRrHzxiLbQmqzcTxSd428bQrkmF5sDF
- connect to ardrone:  `roslaunch ardrone_autonomy ardrone.launch`
- run image_proc for the rectified image of the drone: `ROS_NAMESPACE=/ardrone/front rosrun image_proc image_proc`
- if you want to look through the drone eye: `rosrun image_view image_view image:=/ardrone/front/image_rect_color`
- loading and running the model (it takes about two minutes to load the model): `python best_take_drone_images.py`

## Requirements
Tip: The easiest way to install the following libraries is by creating a virtual environment.

- `python 3.5.2` if you want to train your own network
- `python 2.7.12` for getting the images and pass them through the network
- `tensorflow 1.14.0`
- `keras 2.3.0`
- `OpenCV 3.3.1-dev` is the one that came with ros kinetic

## Vanishing point App

In order to run the app you would need to:
- build the two nodes from Vanihing_Point, I think that `catkin_make` should do it
- connect to ardrone: `roslaunch ardrone_autonomy ardrone.launch`
- run image_proc for the rectified image of the drone: `ROS_NAMESPACE=/ardrone/front rosrun image_proc image_proc`
- run the control: `rosrun drone_control drone_control_node`
- run the node that process the images:  `rosrun CorridorFlyControl CorridorFlyControl`

## Ardrone Setup

1. Install Ubuntu 16.04 LTS

- make a bootable stick with Ubuntu 1604 from here:

	http://releases.ubuntu.com/16.04/

2. Install ROS Kinetic(with Gazebo 7) following the instructions from here:

	http://wiki.ros.org/kinetic/Installation/Ubuntu

3. Install the simulator (tum simulator) an follow the instructions from here:

	https://github.com/angelsantamaria/tum_simulator

- if you want to control the drone using keyboard:

	http://wiki.ros.org/teleop_twist_keyboard
	
4. Install tum_ardrone (for the real drone), which also needs ardrone_autonomy:

	http://wiki.ros.org/tum_ardrone

**Other useful settings and commands:**
- creating a virtual environment: `virtualenv -p <python_version> .venv_name_of_environment`

- if working with VS Code this might help you using the debugger(if you have problems):
	
	https://bytefreaks.net/programming-2/cc-how-do-you-set-gdb-debug-flag-g-with-cmake

# Setup for Jetson Nano

This is if you want to work with a Nvidia Jetson Nano board, also I think that the setup beneath, might also work on Ubuntu 18, not only on Jetson, but is only working in a docker container, so follow these steps:

1. Install docker image: 

	`sudo docker pull ros:kinetic-robot-xenial` 
	
	`docker run -it ros:kinetic-robot-xenial`
	
	`docker ps -l    #to see your container name`
	
	`docker exec -it <your_container_name_here> bash`
	
	`source /opt/ros/<distro>/setup.bash`
	
	`apt-get update`
	You can see all this commands explained here: http://wiki.ros.org/docker/Tutorials/Docker

2. Download the ardrone_autonomy package from this repository https://github.com/dsapandora/ardrone_autonomy :

	`mkdir -p catkin_ws/src`
	
	`cd catkin_ws/src`
	
	`git clone https://github.com/dsapandora/ardrone_autonomy.git`
	
	`apt-get update`
	
	`cd ../`
	
	`rosdep install --from-paths src --ignore-src -r -y`
	
	`catkin_make`
	
3. After the error with `bswap` appears, edit this file: ` devel/src/ardronelib/ARDroneLib/VP_SDK/VP_Os/linux/intrin.h` and replace:
```
static INLINE uint32_t _byteswap_ulong(uint32_t value)
{
   __asm("bswap %0":
     "=r" (value):
     "0" (value));     
  return value;
}
```


with this:
```
static INLINE uint32_t _byteswap_ulong(uint32_t value)
{
  int32_t tmp;

  __asm __volatile(
    "eor        %1, %2, %2, ror #16\n"
    "bic        %1, %1, #0x00ff0000\n"
    "mov        %0, %2, ror #8\n"
    "eor        %0, %0, %1, lsr #8"
    : "=r" (value), "=r" (tmp)
    : "r" (value)
  );

  return value;
}
```
and comment this line: `_BitScanReverse(&index, code);`

 4. After you do this if you try to give it a `catkin_make` it should appear the `mov` error which can be resolved simply just by replacing this line (from the same file as above):

`"mov        %0, %2, ror #8\n"`

with:

`"mov        %0, %2\n"`

and now should work just fine.
 5. In order to save the new package added to this container, from a different terminal do this:
 	`sudo docker commit <ID of container on which you worked> <a custom name for your new image>`
	Now everytime you run this image you will have the ardrone_autonomy package installed.

**Other usefull (docker) commands:**

`sudo docker ps -a  	            # Show all containers (default shows just running)`

`sudo docker start <container_name> #starting a container`
 
`sudo docker run -it -v /data --name <container_name> bash #launch and create a /data volume`
 
`sudo docker images            # Shows all images `
