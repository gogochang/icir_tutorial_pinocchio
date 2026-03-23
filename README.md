# icir_tutorial_pinocchio
CNU icir lab tutorial package for pinocchio library and mujoco simulation

All source codes are opimized for ROS-Noetic.

## 1. Install
# required installation dependencies
```bash
sudo apt install git curl cmake-curses-gui 
sudo apt install ros-noetic-moveit-visual-tools
sudo apt install nlohmann-json3-dev 
sudo apt install ros-noetic-moveit 
sudo apt install libqt53dextras5
sudo apt install qtbase5-private-dev
```

# install pinocchio : add robotpkg apt repository (https://stack-of-tasks.github.io/pinocchio/download.html)
```bash
sudo apt install -qqy lsb-release gnupg2 curl
sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list

sudo apt update 

sudo apt install -qqy robotpkg-py3*-pinocchio
sudo apt install robotpkg-py38-eigenpy
sudo apt install robotpkg-eiquadprog

echo "export PATH=/opt/openrobots/bin:$PATH" >> ~/.bashrc
echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
echo "export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:/opt/openrobots/lib/python3/dist-packages:$PYTHONPATH" >> ~/.bashrc # Adapt your desired python version here
echo "export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH" >> ~/.bashrc
```

# mujoco license
```bash
#ctrl+h to see hidden folder
mkdir .mujoco 
#copy mjkey.txt to .mujoco folder
```

## 2. Prerequisites
```bash
git clone https://github.com/iCIRLab/icir_gen3_robot_description.git
git clone https://github.com/iCIRLab/icir_mujoco_ros.git 
git clone https://github.com/iCIRLab/icir_tutorial_pinocchio.git
```

## 3. Run
```bash
roslaunch icir_tutorial_pinocchio icir_tutorial_pinocchio_simulation.launch 

from terminal, press 'h' to move the robot to the home pose
from terminal, press 'a' to move the robot to another pose

from terminal, press 'k' to move the robot to -0.05m in global z direction
from terminal, press 'v' to enable cartesian impedance control in translational direction
```

## 4. Tutorial assignments
### 4.1. Assignment 3-1: Custom Motion in Joint Space 
![simplescreenrecorder-2026-03-21_15 08 05-2026-03-21_15 33 06-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/dd207a7f-2ec4-44c9-ab45-1affd3ccc514)

### 4.1. Assignment 3-2: Translational Jog in Task Space
![simplescreenrecorder-2026-03-21_15 08 05-2026-03-21_15 34 29-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/69f3d095-d953-4409-a76b-e20d15b7ebb8)

### 4.2. Assignment 3-3: Rotational Jog in Task Space
![simplescreenrecorder-2026-03-21_15 08 05-2026-03-21_15 36 24-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/43bb5cca-d9fe-437f-b677-f24caee99b31)

### 4.2. Assignment 3-4: Sine Motion in Task Space
![simplescreenrecorder-2026-03-21_15 08 05-2026-03-21_15 37 37-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/bfefa8cf-32f4-4e50-b822-278f408d2c49)

### 4.3. Assignment 3-5 & 3-6: Pick-and-Place of Box and Cup Models in MuJoCo
![simplescreenrecorder-2026-03-21_15 08 05-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/5341471b-4fce-4dba-bdc0-c2bbe3c97088)

## 5. Advanced Projects
### 5.1 SAM-based Interactive Pick-and-Place: Click-to-Pick and Lift in MuJoCo
![simplescreenrecorder-2026-03-21_15 08 05-2026-03-21_23 04 46-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/76eb2606-4997-4716-95c2-6a9971c80527)
