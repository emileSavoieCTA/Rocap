# Table of Contents

- [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
- [Install Docker](#install-docker)
  - [Recommendations](#recommendations)
- [Initial Setup](#initial-setup)
- [Usage](#usage)
- [Contact](#contact)
- [Known Issues](#known-issues)

# Introduction

This repository is built with [Docker](https://docs.docker.com/?_gl=1*jirhkt*_gcl_aw*R0NMLjE3Mjk3MTE0OTEuRUFJYUlRb2JDaE1JMVB1ZHRwMmxpUU1WRFU3X0FSMDFfZzVPRUFBWUFTQUFFZ0tsOFBEX0J3RQ..*_gcl_au*MjEwMDM5OTQxNS4xNzI3MTExOTkz*_ga*MzI1MTgxNzg4LjE3MjcxMTA2OTA.*_ga_XJWPQMJYHQ*MTczNDAyOTUzOS4yMy4xLjE3MzQwMjk3ODUuNDcuMC4w) to allow the code to run on a Ubuntu 22.04 image with ROS2 Humble LTS and all necessary dependencies, regardless of the user's operating system.

# Install Docker

## Docker Installation

For Docker installation, follow these instructions:
- [Docker Windows](https://docs.docker.com/desktop/setup/windows-install/)

Once Docker is installed, ensure the Docker Desktop application remains open throughout the process.

# Initial Setup

## Setup WSL

To access UI applications from a Dev Container on Windows, follow these steps:

1. Install the Ubuntu application from the [Microsoft Store](https://apps.microsoft.com/detail/9pdxgncfsczv?hl=en-US&gl=US)

2. Open a Command Prompt (Windows Key + "cmd")

    <img src="images/cmd.png" alt="Project Logo" width="1000" />

3. Verify Ubuntu installation:
   ```bash
   wsl --list
   ```
   
   ![WSL List](images/wsl_list.png)

4. If Ubuntu is not the default, set it as default:
   ```bash
   wsl --set-default Ubuntu
   wsl --shutdown
   ```

   > **Note**: If Ubuntu is not in the list, additional configuration may be required.

5. Open Ubuntu terminal:
   ```bash
   bash
   ```

6. Navigate to the "docker" folder and run Docker Compose:
   ```bash
   cd /path/to/docker/folder
   docker-compose -f docker-compose.yml -f docker-compose.override.wsl.yml up
   ```

   ![Docker Compose](images/compose.png)

## Setup Repository in Visual Studio Code

1. Open the source code in VSCode, selecting the folder with the `.devcontainer` folder visible:

   ![VSCode Folder](images/folder_vscode.png)

2. Install required extensions:
   
   ![Docker Extension](images/docker.png)
   ![Dev Container Extension](images/dev_container.png)

3. Reopen the workspace in a Dev Container:
   - Use the popup upon opening
   
     ![Reopen Container Popup](images/reopen_container.png)

   - Or manually:
     - Click the blue button in the bottom left
     
       ![Blue Button](images/blue_button.png)
     
     - Select "Reopen in Container"
     
       ![Select Reopen](images/select_reopen.png)

4. Once in the Dev Container, run the build script:
   ```bash
   ./build.bash
   ```

# Usage

The ROS2 environment for the Rocap supports multiple simulation configurations:

1. Complete simulation (odometry and lidar simulated):
   ```bash
   ros2 launch rocap_ros rocap.launch.py
   ```

2. API bridge with Lidar integration:
   ```bash
   ros2 launch rocap_ros rocap.launch.py sim:=false velodyne:=true
   ```

   ![Rocap Simulation](images/Rocap_sim.png)

> **Note**: Other configurations are possible but may require additional testing.

# Known Issues

[Placeholder for known issues section]

# Contact

This code was developed by the Cégep Édouard-Montpetit under grant 22CTA034.