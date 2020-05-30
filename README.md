# Open REALM ROS1 Bridge

This is the repository for the OpenREALM ROS1 bridge. For informations and updates on the main library, please refer to:

https://github.com/laxnpander/OpenREALM.git

## Prerequisites


| OS         | ROS Distribution | Build Status |
|:----------:|:----------------:|:------------:|
|Ubuntu 16.04| ROS Kinetic      | [![Build Status](https://travis-ci.org/laxnpander/OpenREALM_ROS1_Bridge.svg?branch=master)](https://travis-ci.org/github/laxnpander/OpenREALM_ROS1_Bridge) |
|Ubuntu 18.04| ROS Melodic      | [![Build Status](https://travis-ci.org/laxnpander/OpenREALM_ROS1_Bridge.svg?branch=master)](https://travis-ci.org/github/laxnpander/OpenREALM_ROS1_Bridge) |

For ROS installation please refer to: http://wiki.ros.org/

Other dependencies are installed using the  ```install_deps.sh``` script:
```sh
chmod u+x install_deps.sh
./install_deps.sh
```
Do not proceed to the next step before 
you executed this script.

## Optional Dependencies

CUDA for stereo reconstruction with plane sweep lib

-> Refer to https://developer.nvidia.com/cuda-downloads?target_os=Linux

Please note, that installing CUDA can sometimes be troublesome. If you are facing an error like 
```sh
*fatal error: cuda_runtime.h: No such file or directory*
```
often times adding the CUDA directory to the .bashrc does the trick. If you use CUDA 9.0 for example, you should 
```sh
echo 'export CPATH=/usr/local/cuda-9.0/include:$CPATH' >> ~/.bashrc 
```

## Installation

In the following I assume you already called the install_deps.sh and therefore have all dependencies.

Linux (both Ubuntu 16.04 and 18.04)

```sh
# Create and init a catkin workspace
mkdir -p catkin_ws/src
cd catkin/src

# Make sure you are in your catkin_ws, not src
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Quick Start

**Step 1:**
Download the test dataset:

https://drive.google.com/open?id=1-2h0tasI4wzxZKLBbOz3XbJ7f5xlxlMe

**Step 2:**
Unzip the dataset with a tool of your choice, e.g.
```sh
tar -xvzf open_realm_edm_dataset.tar.gz
```

**Step 3:**
We provided as well a set of configuration files in realm_ros/profiles as the corresponding launch files in 
realm_ros/launch to run the test dataset. The only thing you have to do is modify the path in the launch file:
```sh
node pkg="realm_ros" type="realm_exiv2_grabber" name="realm_exiv2_grabber" output="screen"
    param name="config/id" type="string" value="$(arg camera_id)"/>
    param name="config/input" type="string" value="PUT THE TEST DATASET'S ABSOLUTE PATH HERE"/>
    param name="config/rate" type="double" value="10.0"/>
    param name="config/profile" type="string" value="alexa"/>
/node
```
Note: The exiv2 grabber node reads images and exiv2 tags from the provided folder and publishes them 
for the mapping pipeline.

**Step 4:**
Launch the pipeline you want to use.

- GNSS only mapping:
```sh
roslaunch realm_ros alexa_gnss.launch
```

- 2D mapping with visual SLAM:
```sh
roslaunch realm_ros alexa_noreco.launch
```

- 2.5D mapping with visual SLAM and surface reconstruction:
```sh
roslaunch realm_ros alexa_reco.launch
```