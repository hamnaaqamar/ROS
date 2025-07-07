
# PX4 SITL + MAVROS + ROS Noetic Setup (Ubuntu 20.04 on VirtualBox)

This guide walks you through setting up PX4 drone simulation (with **JMAVSim**), MAVROS, and ROS Noetic on **Ubuntu 20.04**, using **VirtualBox**. Ideal for beginners testing drone missions without hardware.

---

## Step 1: VirtualBox + Ubuntu Installation

### 1.1 Install VirtualBox
- Download from: https://www.virtualbox.org/wiki/Downloads
- Choose **Windows hosts** if you’re on Windows.

### 1.2 Download Ubuntu 20.04 ISO
- Get from: https://releases.ubuntu.com/20.04/

### 1.3 Create Virtual Machine
In VirtualBox:
- **New VM** → Name: `Ubuntu-ROS`
- **Type:** Linux
- **Version:** Ubuntu (64-bit)
- **Memory:** At least 4096 MB (6144 MB+ recommended)
- **Disk:** VDI, dynamically allocated, 40 GB
- Mount the Ubuntu ISO and install Ubuntu

---

## Step 2: First Boot + Terminal Fix (if needed)
After installation:
- Log into Ubuntu
- Open Terminal:  
  Press `Ctrl + Alt + T`  
  OR right-click Desktop → "Open Terminal"
- If terminal fails:
  ```bash
  sudo apt update
  sudo apt install gnome-terminal
  ```

---

## Step 3: Install ROS Noetic (Ubuntu 20.04)

### 3.1 Add ROS Repository
```bash
sudo apt update
sudo apt install curl gnupg lsb-release
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 3.2 Add ROS GPG Key
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 3.3 Install ROS Desktop Full
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

### 3.4 Initialize `rosdep`
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### 3.5 Auto-source ROS
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 4: Install MAVROS

### 4.1 Install GeographicLib
```bash
sudo apt install geographiclib-tools
```

### 4.2 Install MAVROS
```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```

### 4.3 Install GeographicLib Datasets
```bash
sudo geographiclib-get-geoids egm96-5
sudo geographiclib-get-gravity egm96
sudo geographiclib-get-magnetic igrf
```

---

## Step 5: Clone and Setup PX4 with JMAVSim

### 5.1 Install Git (if not installed)
```bash
sudo apt update
sudo apt install git
```

### 5.2 Clone PX4 Autopilot Repo (Stable Version)
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git fetch
git checkout v1.13.3
git submodule update --init --recursive
```

### 5.3 Install PX4 Dependencies
```bash
bash ./Tools/setup/ubuntu.sh
```

### 5.4 Build PX4 with JMAVSim
```bash
make px4_sitl jmavsim
```

> If you see an error like `ant: command not found`, install it:
```bash
sudo apt install ant openjdk-11-jdk
```

---

## Step 6: Run PX4 + MAVROS + ROS Mission

### Terminal 1: Run PX4 SITL (Simulator)
```bash
cd ~/PX4-Autopilot
make px4_sitl jmavsim
```

Wait until you see messages like:
```
INFO  [simulator] Waiting for simulator to accept connection on TCP port 4560
```

### Terminal 2: Start ROS Master
```bash
source /opt/ros/noetic/setup.bash
roscore
```

### Terminal 3: Run MAVROS
```bash
source /opt/ros/noetic/setup.bash
rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14557
```

### Terminal 4: Run Your Mission Code
```bash
source ~/catkin_ws/devel/setup.bash
rosrun your_package_name your_script.py
```

---

## Final Checklist

- [x] Ubuntu 20.04 installed in VirtualBox
- [x] ROS Noetic installed and working
- [x] MAVROS installed with GeographicLib
- [x] PX4 (v1.13.3) built with JMAVSim
- [x] MAVROS and PX4 communicate via UDP
- [x] Can run custom mission code using ROS

---

## Helpful Commands

| Command | Description |
|--------|-------------|
| `roscore` | Start ROS Master |
| `rosrun [pkg] [node]` | Run a ROS node |
| `roslaunch [pkg] [file]` | Launch file-based setup |
| `rostopic list` | View active ROS topics |
| `rostopic echo /topic_name` | View data on a topic |
| `rqt_graph` | Visual node-topic graph |

---



