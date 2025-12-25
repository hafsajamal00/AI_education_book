---
sidebar_position: 1
---

# Installation

This guide will help you install ROS2 on your system.

## System Requirements

- Ubuntu 22.04 (Jammy Jellyfish) or Windows 10/11
- At least 5GB of free disk space
- Internet connection for downloading packages

## Installing on Ubuntu

1. Set locale:
   ```bash
   locale-gen en_US.UTF-8
   ```

2. Add the ROS2 apt repository:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. Install ROS2:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

4. Source the setup script:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

## Installing on Windows

1. Install Chocolatey package manager
2. Install Visual Studio with C++ support
3. Download and run the ROS2 installer for Windows

## Verification

After installation, verify that ROS2 is properly installed:

```bash
source /opt/ros/humble/setup.bash  # On Ubuntu
ros2 --version
```

You should see the version of ROS2 that you installed.