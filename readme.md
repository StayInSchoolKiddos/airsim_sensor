#  Airsim JSBsim
This repo is used to wrap and interface with ROS 2 using Airsim's camera and lidar, this is done since the airsom_ros_pkgs require you to be in a Drone or Car. There is no support for Computer Vision Mode

## Install dependencies
- Make sure to install Airsim and Unreal Engine 
- If you are interfacing this with ROS with WSL2 clone this specific Airsim repo: https://github.com/StayInSchoolKiddos/AirSim and compile the code 

## Virtual environment
Before you start developing set up your virtual environment and install your Python dependencies
```
virtualenv venv #do this command once
source venv/bin/activate #activate your virtual environment
pip install -r requirements.txt
```
