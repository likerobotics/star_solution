# USER COMMANDS

Go to your ros2 Jazzy workspace and place the package in src folder.


some useful commands:
```
cd /home/likerobotics/ros2_jazzy


colcon build --symlink-install --packages-select star_solution

source install/setup.bash

ros2 launch star_solution bring_up.launch.py

rm -rf build install log

```



## Installations

```
sudo apt install ros-jazzy-cartographer ros-jazzy-cartographer-ros ros-jazzy-cartographer-ros-msgs

```
Chek the required packages...
ros2 pkg list | grep cartographer




## Vistual env and external packages

I have created env for external pakckages to be possible use.
```
python3 -m venv ~/ros2_jazzy/ros2_open3d_env

sudo apt install python3-sklearn

pip install "numpy<2.0" open3d

```