
# JMAVSim + PX4 + ROS Noetic + MAVROS + Custom Mission Script Setup (Ubuntu 20.04 on VirtualBox)

---

## Step 1: VirtualBox + Ubuntu Setup

### 1. Install VirtualBox
- Download from: https://www.virtualbox.org/wiki/Downloads
- Choose "Windows hosts" (if you're using Windows).

### 2. Download Ubuntu 20.04 ISO
- Get the ISO: https://releases.ubuntu.com/20.04/

### 3. Create Virtual Machine in VirtualBox
- Name: `Ubuntu-ROS`
- Type: `Linux`, Version: `Ubuntu (64-bit)`
- Memory: `4096 MB (minimum)` (Recommended: `6144 MB+`)
- Hard Disk: `VDI`, Dynamically Allocated, Size: `40 GB`
- Mount Ubuntu ISO → Install Ubuntu

---

##  Step 2: First Ubuntu Boot & Terminal Setup

After installation:
- Log in → Open Terminal via `Ctrl + Alt + T` or right-click → Open Terminal

If terminal fails:
```bash
sudo apt update
sudo apt install gnome-terminal
```

---

##  Step 3: ROS Noetic Installation

```bash
sudo apt update
sudo apt install curl gnupg lsb-release

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop-full

sudo apt install python3-rosdep
sudo rosdep init
rosdep update

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 4: Install MAVROS

```bash
sudo apt install geographiclib-tools
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras

sudo geographiclib-get-geoids egm96-5
sudo geographiclib-get-gravity egm96
sudo geographiclib-get-magnetic igrf
```

---

## Step 5: PX4 & JMAVSim Setup

### Clone PX4 Repo

```bash
sudo apt install git
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```

### Checkout a Stable Version

```bash
git fetch
git checkout v1.13.3
git submodule update --init --recursive
```

### Install Dependencies

```bash
bash ./Tools/setup/ubuntu.sh
```

### Install `ant` (required for JMAVSim)

```bash
sudo apt install ant openjdk-11-jdk
```

---

## Step 6: Build PX4 with JMAVSim

```bash
cd ~/PX4-Autopilot
make px4_sitl_default jmavsim
```

This opens PX4 + JMAVSim in software-in-the-loop simulation.

---

## Step 7: Create ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

##  Step 8: Create Custom Package

```bash
cd ~/catkin_ws/src
catkin_create_pkg tekno_mission3_pkg rospy std_msgs geometry_msgs mavros_msgs
```

---

## Step 9: Add Mission Script

```bash
mkdir -p ~/catkin_ws/src/tekno_mission3_pkg/scripts
cp [your_mission_script].py ~/catkin_ws/src/tekno_mission3_pkg/scripts/mission3.py
chmod +x ~/catkin_ws/src/tekno_mission3_pkg/scripts/mission3.py
```

---

## Step 10: Build Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

##  Step 11: Run the Full Simulation

### In 3 terminals:

**Terminal 1:** Start ROS Master
```bash
source /opt/ros/noetic/setup.bash
roscore
```

**Terminal 2:** Launch PX4 SITL + JMAVSim
```bash
cd ~/PX4-Autopilot
make px4_sitl_default jmavsim
```

**Terminal 3:** Launch MAVROS
```bash
source /opt/ros/noetic/setup.bash
rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14557
```

**Terminal 4 (new):** Run your Mission Script
```bash
source ~/catkin_ws/devel/setup.bash
rosrun tekno_mission3_pkg mission3.py
```

---

##  Helpful ROS Commands

| Command                        | Description                            |
|-------------------------------|----------------------------------------|
| `roscore`                     | Start ROS Master                       |
| `rosrun [pkg] [node]`         | Run a ROS node                         |
| `roslaunch [pkg] [file]`      | Launch file for complex setups         |
| `rostopic list`               | See all active ROS topics              |
| `rostopic echo /topic_name`   | View published messages                |
| `rqt_graph`                   | View graph of active ROS nodes         |

---

## Resources

- ROS Tutorials: https://wiki.ros.org/ROS/Tutorials
- MAVROS Wiki: https://wiki.ros.org/mavros
- PX4 MAVROS Setup: https://docs.px4.io/master/en/ros/mavros_offboard.html

---

## Final Checklist

✅ Ubuntu 20.04 on VirtualBox installed  
✅ ROS Noetic & MAVROS configured  
✅ PX4-Autopilot & JMAVSim running  
✅ Mission script tested via ROS node  
✅ MAVROS talks to PX4 (via UDP)  
✅ You can now simulate full delivery missions! 
