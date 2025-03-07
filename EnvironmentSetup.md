# Docker Environment Setup
Getting a Docker container that does everything we need is not trivial. The following steps is what I was able to get working for my NVIDIA Jetson Orin Nano.

1) Follow system setup (through at least the “Relocating Docker Data Root” step) from [this link](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md). If you have NVME storage to add, I found [this tutorial](https://www.digitalocean.com/community/tutorials/how-to-partition-and-format-storage-devices-in-linux) helpful. Also, be sure to add the following line to etc/docker/daemon.json:
```
"data-root": "/mnt/docker"
```

2) Choose a base container. I like l4t-ml since it contains cuda enabled pytorch, cuda enabled tensorflow2, and cuda enabled opencv. The full list of containers can be found [here](https://github.com/dusty-nv/jetson-containers/tree/master). Now, run the container, e.g.:
```
jetson-containers run $(autotag l4t-ml)
```

3) Now we can install ROS. For example, for ROS Noetic, follow the instructions [here](https://wiki.ros.org/noetic/Installation/Ubuntu). IMPORTANT: Install the **ROS-Base** version. The other version will attempt to modify the opencv, which we cannot do! Thus, install the bare bones version and the selectively install additional packages we may need. The following is also useful to put in your ~/.bashrc file:
```
source /workspace/catkin_ws/devel/setup.bash
export ROS_IP=192.168.XX.XX
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROBOT_ID=*insert robot id here*
```

4) Install any additional ros packages you need, e.g.
```
sudo apt install ros-noetic-PACKAGE
```
I installed the following:
- tf2-msgs
- tf
- gmapping
- diagnostic-updater

For the camera to work, I find it necessary to first run:
```
apt-get purge -y '*opencv*'
```
Then, follow instructions at [this link](https://github.com/satomm1/ros_astra_camera), with the following exception: run these lines outside of the docker container
```
./scripts/create_udev_rules
sudo udevadm control --reload && sudo  udevadm trigger
```

5) Install other python packages you need. IMPORTANT: Any package which has an opencv-python dependency must be sure not to modify the opencv-python package already installed from the original container. Updating will break the package! Some packages I install include:
- spidev
- confluent-kafka
- rospy-message-converter
- pyignite
- matplotlib
- scipy
- ultralytics****be careful with opencv-python here!

6) Commit the docker container so you can use it later, e.g.:
```
docker commit c3f279d17e0a ml_ros:latest
```

7) Now, to run the container, we can use the command:
```
jetson-containers run $(autotag ml_ros:latest)
```
You can include any of the usual docker commands with this. For example, my full command is:
```
jetson-containers run -v ~/workspaces/catkin_ws:/workspace/catkin_ws -v ~/gemini_api:/gemini_code -v /dev/bus/usb:/dev/bus/usb -i --device=/dev/ttyUSB0 --device=/dev/spidev0.0 --rm --privileged --name ros_noetic $(autotag ml_ros:latest)
```
Alternatively, use docker run directly:
```
sudo docker run --runtime nvidia --network=host -v ~/workspaces/catkin_ws:/workspace/catkin_ws -v ~/gemini_api:/gemini_code -v /dev/bus/usb:/dev/bus/usb -it --device=/dev/ttyUSB0 --device=/dev/spidev0.0 --rm --privileged --name ros_noetic ml_ros:latest
```

If everything has gone according to plan, you should now have a docker container which has ROS, cuda enabled pytorch/tensorflow, and everything else you might need!

Now, you can clone the various repos into the catkin_ws/src directory. Before calling `catkin_make` for the first time, you will need to source the setup file: `source /opt/ros/noetic/setup.bash`.