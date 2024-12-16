# Table of contents

- [Table of Content](#table-of-content)
- [Introduction](#introduction)
- [Install Docker](#install-docker)
  - [Recommendations](#recommendations)

# Introduction

This page acts as a guide for setup and usage of the ROS package destined to the Rbot9 ROCAP

# Initial Setup

## Docker Installation
This repository is built with [Docker](https://docs.docker.com/?_gl=1*jirhkt*_gcl_aw*R0NMLjE3Mjk3MTE0OTEuRUFJYUlRb2JDaE1JMVB1ZHRwMmxpUU1WRFU3X0FSMDFfZzVPRUFBWUFTQUFFZ0tsOFBEX0J3RQ..*_gcl_au*MjEwMDM5OTQxNS4xNzI3MTExOTkz*_ga*MzI1MTgxNzg4LjE3MjcxMTA2OTA.*_ga_XJWPQMJYHQ*MTczNDAyOTUzOS4yMy4xLjE3MzQwMjk3ODUuNDcuMC4w). This allows the code to run on a Ubuntu 22.04 image with ROS2 Humble LTS and all necessary dependencies regardless of the user's operating system. 

For Docker installation instructions, follow one of the following links:
- [Docker Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
- [Docker Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

## Setup Repository
Once Docker is installed, go to your IDE of choice and install the Docker extension. This will allow you to develop from within the container. Once setup, follow the following instructions to clone the repository.

1. Open git bash and run the following command at the desired location

```
https://github.com/cegepmontpetit/22CTA034_ROS2_Package.git
```

# Usage
Once in a working container, the code is now ready to be used.

## Using the Rocap in simulation
To launch the Rocap in a Gazebo simulation run the following command in container terminal:

```bash
ros2 launch rocap_ros rocap.launch.py
```
This will launch all necessary nodes to operate the Rocap in its simulation environment. The process should yield the following result after a couple of seconds (up to 30s):

<img src="images/Rocap_sim.png" alt="Project Logo" width="500" />

If it does not, please refer to the [Know Issues](#known-issues) tab for more information.



## Using the real Rocap
Before using the real Rocap, ensure you have completed the following steps:
1. Request a signed_permissions.json file from the Rocap administrator.
2. Place the signed_permissions.json file in the permissions folder.
3. Determine the IP address and communication port of the Rocap you want to control.

Once you have completed the above steps, you can start the API bridge to control the Rocap through ROS using the following command:

```bash
ros2 launch rocap_ros rocap_bridge base_url:='http://<rocapIP>:<rocapPort>'
```

# Contact
This code was written by the Cégep Édouard-Montpetit under grant 22CTA034

