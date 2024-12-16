#!/bin/bash

# Function to clone or pull a git repository
clone_or_pull() {
    local repo_url=$1
    local branch=$2
    local dir_name=$3

    if [ -d "$dir_name" ]; then
        echo "Directory '$dir_name' exists. Pulling the latest version..."
        git -C "$dir_name" pull origin "$branch"
    else
        echo "Directory '$dir_name' does not exist. Cloning the repository..."
        git clone -b "$branch" "$repo_url" "$dir_name"
    fi
}

cd ${WORKSPACE}
mkdir src
cd src

# Clone or pull the repositories
clone_or_pull "https://github.com/emileSavoieCTA/Rocap.git" "rocap_ros"
# clone_or_pull "https://github.com/cegepmontpetit/ROS2_CTA.git" "$ROS_DISTRO" "cta_gazebo_part"

cd ${WORKSPACE}
rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source ${WORKSPACE}/install/local_setup.bash" >> ~/.bashrc

pipx run openapi-python-client generate --path ${WORKSPACE}/src/rocap_ros/rocap_ros/node/api_bridge/openapi.json --output-path ${WORKSPACE}/rocap-rest-api-client --overwrite
cd ${WORKSPACE}/rocap-rest-api-client
pipx run poetry install
