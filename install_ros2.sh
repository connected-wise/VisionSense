#!/bin/bash
# ROS2 Humble Installation Script for Ubuntu 22.04

echo "Starting ROS2 Humble installation..."

# 1. Setup locale
echo "Setting up locale..."
apt update && apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Setup sources
echo "Setting up ROS2 repositories..."
apt install -y software-properties-common
add-apt-repository universe -y
apt update && apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS2 packages
echo "Installing ROS2 Humble..."
apt update
apt install -y ros-humble-desktop python3-colcon-common-extensions

# 4. Install additional dependencies
echo "Installing additional dependencies..."
apt install -y python3-rosdep python3-vcstool

# 5. Initialize rosdep
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
su -c "rosdep update" $SUDO_USER

# 6. Setup environment for user
echo "Setting up environment..."
echo "source /opt/ros/humble/setup.bash" >> /home/$SUDO_USER/.bashrc
chown $SUDO_USER:$SUDO_USER /home/$SUDO_USER/.bashrc

echo "ROS2 Humble installation complete!"
echo "Please run 'source ~/.bashrc' or open a new terminal to use ROS2"