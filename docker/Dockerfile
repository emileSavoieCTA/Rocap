FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/home
ENV WORKSPACE=${HOME}/rocap_ws

ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install the required ROS packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  ssh \
  curl \
  git \
  pipx \
  python3-pip \
  xterm \
  tmux

WORKDIR ${WORKSPACE}

RUN curl -sSL https://install.python-poetry.org | python3 -

ENV POETRY_VIRTUALENVS_CREATE=false

RUN pipx install ruff

RUN rosdep fix-permissions && rosdep update --rosdistro=$ROS_DISTRO

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y \
  gazebo \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs

RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-rtabmap-ros \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-gazebo-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-octomap \
  ros-${ROS_DISTRO}-tf-transformations \
  python3-transforms3d

RUN rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Set the working directory
WORKDIR ${WORKSPACE}
COPY build.bash ./build.bash
RUN chmod +x ./build.bash
