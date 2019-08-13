# Ardrone Setup

1. Install Ubuntu 16.04 LTS

- make a bootable stick with Ubuntu 1604 from here:

	http://releases.ubuntu.com/16.04/

2. Install ROS Kinetic(with Gazebo 7) following the instructions from here:

	http://wiki.ros.org/kinetic/Installation/Ubuntu

3. Install the tum simulator an follow the instructions from here:

	https://github.com/angelsantamaria/tum_simulator

- if you want to control the drone using keyboard:

	http://wiki.ros.org/teleop_twist_keyboard


Other useful settings:

- if working with VS Code this might help you using the debugger:
	https://bytefreaks.net/programming-2/cc-how-do-you-set-gdb-debug-flag-g-with-cmake

# Setup for Jetson Nano

It is only working in a docker, so follow thes steps:

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

`static INLINE uint32_t _byteswap_ulong(uint32_t value)
{
   __asm("bswap %0":
     "=r" (value):
     "0" (value));     
  return value;
}`


with this:

`static INLINE uint32_t _byteswap_ulong(uint32_t value)
{
  int32_t tmp;`

 ` __asm __volatile(
    "eor        %1, %2, %2, ror #16\n"
    "bic        %1, %1, #0x00ff0000\n"
    "mov        %0, %2, ror #8\n"
    "eor        %0, %0, %1, lsr #8"
    : "=r" (value), "=r" (tmp)
    : "r" (value)
  );`

  `return value;
}`

and comment this line: `_BitScanReverse(&index, code);`

 4. After you do this if you try to give it a `catkin_make` it should appear the `mov` error which can be resolved simply just by replacing this line (from the same file as above):

`"mov        %0, %2, ror #8\n"`

with:

`"mov        %0, %2\n"`

and now should work just fine. 
