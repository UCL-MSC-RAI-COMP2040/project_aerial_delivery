# Project Aerial Delivery

This tutorial project simulates a drone delivery flying task

This project is based on aerostack2 drone flying framework which uses ROS2 but has a nice python api

<!-- Tutorial is [here](https://ucl-delta.github.io/project_gazebo_aruco/) -->


**IN PROGRESS**

To ensure that the plugin builds, please place this project also in your ros2 src folder along with aerostack2 and colcon build it. Then run the launch_as2 as before from this repository. 

TODOS:
- Custom gripping/pickup: either use aerostack2 gripper, or use the fake gripper plugin in sim_config/plugins

README.md needs updating


## Usage

There are three ways you can run this project depending on your situation or operating system. 
- Linux: All 3 ways will work
- Windows: Docker and Docker with VNC (You will need the WSL2 backend)
- Max OSX: Docker with VNC

Note that you will need at least 20Gb free on your system to avoid lockup. Graphics card and 16Gb RAM is recommended. 

> **Windows** You will need to install WSL2 for Docker and these docker containers to work. [Instructions Here](https://learn.microsoft.com/en-us/windows/wsl/install)

See Documentation for detailed instructions

### Local

Install Ubuntu 22.04, ROS2 Humble, Ignition Gazebo Fortress as per their instructions

Setup and build Aerostack2 (we use version 1.1.2)

Get this project locally

```
mkdir -p ~/aerostack2_ws/src
cd ~/aerostack2_ws/src
git clone https://github.com/ucl-delta/project_gazebo_aruco.git
```

Run the example using 

```
./launch_as2.bash -s -t
```

### Docker

Ensure Docker or Docker Desktop is installed on your machine

First your will need to clone this project somewhere (doesn't need to be in a ros2 workspace)

```
git clone https://github.com/ucl-delta/project_gazebo_aruco.git
```

To build and/or run the container run the script

This container is based on Ubuntu 22.04, ROS2 Humble and Ignition Gazebo Fortress

```
./docker/docker_start.bash
```

After building for a while, this will drop you inside the docker container.

The container will have live mounted this project into `/ros2/project_gazebo_aruco` so that any changes made to this repository outside of the container will be reflected inside. 

Inside the container, navigate to that repository and run the example.

```
cd /ros2/project_gazebo_aruco
./launch_as2.bash -s -t
```

> Note that you can utilise a GPU if you install the `nvidia-container-toolkit`. Pass the `-nvidia` argument to `docker_start.bash`

### Docker with VNC

This will enable all of the ROS2 to run standalone with no outside network connections. It makes use of a Virtual Network Computing interface to share a the container's desktop GUI with the outside world. In this case your browser! 

Ensure Docker or Docker Desktop is installed on your machine

First your will need to clone this project somewhere (doesn't need to be in a ros2 workspace)

```
git clone https://github.com/ucl-delta/project_gazebo_aruco.git
```

To build and/or run the container run the script

This container is based on Ubuntu 22.04, ROS2 Humble and Ignition Gazebo Fortress

```
./docker/docker_vnc_start.bash
```

After building for a while, this will say that it has started the VNC server

> Note: The first time you build it it may time out. Run the command again and it should comeplete the build. 

Go into a browser and navigate to `https://127.0.0.1:6080` and press `connect`. This will drop you into an ubunut22.04 desktop environment with all the things you need! 

The container will have live mounted this project into `/ros2/project_gazebo_aruco` so that any changes made to this repository outside of the container will be reflected inside. 

Open up a terminal (`terminator`) and navigate to `/ros2/project_gazebo_aruco` to run the example. 

```
cd /ros2/project_gazebo_aruco
./launch_as2.bash -s -t
```

> Note that you can utilise a GPU if you install the `nvidia-container-toolkit`. Pass the `-nvidia` argument to `docker_vnc_start.bash` when running the vnc. 

## Contact

This project is developed by Mickey Li - [email](mickey.li@ucl.ac.uk)

## License

This project is released under permissive BSD 3 License


