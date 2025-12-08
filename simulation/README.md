# AirSim & Gazebo Installation Guide (Ubuntu 22.04, ROS2 Humble)

## AirSim Installation (for ROS2 Humble)

1. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install git cmake build-essential libboost-all-dev python3 python3-pip
   ```

2. **Clone AirSim:**
   ```bash
   git clone https://github.com/ahmedislamfarouk/AirSim.git
   cd AirSim
   ```
   Or clone the official repo:
   ```bash
   git clone https://github.com/microsoft/AirSim.git
   cd AirSim
   ```

3. **Build AirSim:**
   ```bash
   ./setup.sh
   ./build.sh
   ```

4. **ROS2 Humble Bridge:**
   - Install ROS2 Humble: [ROS2 Install Guide](https://docs.ros.org/en/humble/Installation.html)
   - Build ROS2 bridge:
     ```bash
     cd ros2
     colcon build
     source install/setup.bash
     ```

5. **Why use this fork?**
   - This fork ([ahmedislamfarouk/AirSim](https://github.com/ahmedislamfarouk/AirSim)) fixes all installation errors for Ubuntu 22.04 and ROS2 Humble, including build, setup, and ROS2 connection issues. You can use it for a smoother experience.

**Notes:**
- AirSim is used for drone/car simulation and works with ROS2 Humble for robotics projects.
- For Unreal Engine environments, see AirSim docs for extra setup.
- AirSim Docs: https://airsim-fork.readthedocs.io/en/docs/

---

## Gazebo Harmonic + ROS2 Humble + PX4 + QGroundControl Installation (Detailed)

### 1. Install ROS2 Humble (LTS)
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```
- Check ROS2 version:
  ```bash
  ls /opt/ros/
  source /opt/ros/humble/setup.bash
  printenv | grep ROS
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ros2 --version
  ros2 run demo_nodes_cpp talker
  # (Press Ctrl+C to stop)
  ```

### 2. Install Gazebo Harmonic
```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-harmonic
```
- Check Gazebo version:
  ```bash
  gz sim --version
  gz sim
  gz sim shapes.sdf
  # (Press Ctrl+C to stop)
  ```

### 3. Install ROS2 - Gazebo Bridge
```bash
sudo apt-get install ros-humble-ros-gz
```
- Check bridge packages:
  ```bash
  ros2 pkg list | grep ros_gz
  ros2 run ros_gz_bridge parameter_bridge
  ```

### 4. Install PX4 Autopilot
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```
- This installs build toolchain, Python dependencies, simulation tools, MAVSDK dependencies.

### 5. Build PX4 with Gazebo Support
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
- Simple PX4 commands (run in same terminal):
  ```bash
  commander arm -f
  commander takeoff
  commander land
  ```

### 6. Install MAVSDK (Python & C++)
```bash
pip3 install mavsdk
sudo apt install libmavsdk-dev
```
- You might need to add them to environment variables.

### 7. Install QGroundControl
```bash
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
- Download QGroundControl-x86_64.AppImage from the official site.
- Make it executable:
  ```bash
  chmod +x QGroundControl-x86_64.AppImage
  ./QGroundControl-x86_64.AppImage
  ```

**Notes:**
- Gazebo Harmonic is the recommended simulator for ROS2 Humble and PX4.
- Use RViz2 and QGroundControl for visualization and control.
- For more details, see official [Gazebo Docs](https://gazebosim.org/docs/harmonic), [PX4 Docs](https://docs.px4.io/), and [QGroundControl Docs](https://docs.qgroundcontrol.com/).
