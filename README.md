# Project Aerial Delivery

This tutorial project simulates a drone delivery flying task

This project is based on aerostack2 drone flying framework which uses ROS2 but has a nice python api

<!-- Tutorial is [here](https://ucl-delta.github.io/project_gazebo_aruco/) -->


**IN PROGRESS**

To ensure that the plugin builds, please place this project also in your ros2 src folder along with aerostack2 and colcon build it. Then run the launch_as2 as before from this repository. 

TODOS:
- Custom gripping/pickup: either use aerostack2 gripper, or use the fake gripper plugin in sim_config/plugins

README.md needs updating

## Install

### Aerostack2 

> System must be running Ubuntu 22.04 with ROS2 humble and Gazebo Fortress Installed

Please first ensure that **you have installed and our version of aerstack2** within your workspace:

```
cd <my_ws>/src/
git clone https://github.com/UCL-MSC-RAI-COMP2040/aerostack2.git
```

and that it has been built in the workspace root. 
```
colcon build --symlink-install
```

### Aerial Delivery Project


> *NOTE* If on Ubuntu 22.04 running ROS2 Humble you will need to patch a known problem in jsoncpp build system which hasn't yet been backported. Copy the contents of the file `sim_config/gazebo/jsoncpp-namespaced-targets.cmake` to `/usr/lib/x86_64-linux-gnu/cmake/jsoncpp/jsoncpp-namespaced-targets.cmake`
>
> See [link](https://github.com/open-source-parsers/jsoncpp/pull/1435/files) for more info


## Contact

This project is developed by Mickey Li - [email](mickey.li@ucl.ac.uk)

## License

This project is released under permissive BSD 3 License


