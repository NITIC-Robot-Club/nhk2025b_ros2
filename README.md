# nhk2025b_ros2
[![build docker image](https://github.com/NITIC-Robot-Club/nhk2025b_ros2/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/nhk2025b_ros2/actions/workflows/docker_build.yaml)
ros2のwsです

[Document](https://nitic-robot-club.github.io/nhk2025b_ros2)

# install
```bash
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install can-utils
```

# rosbag
```bash
ros2 bag record --all
```

# CANABLE
```bash
sudo nano /etc/udev/rules.d/99-canable.rules
```
```
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="606f", ATTR{serial}=="0058001E5741500C20393541", SYMLINK+="can", \
RUN+="/sbin/ip link set can0 type can bitrate 1000000", \
RUN+="/sbin/ip link set can0 up"

SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="606f", ATTR{serial}=="004A00433136500C2039384D", SYMLINK+="can", \
RUN+="/sbin/ip link set can0 type can bitrate 1000000", \
RUN+="/sbin/ip link set can0 up"
```
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

# Virtual CAN

first
```bash
sudo modprobe vcan
```

start
```bash
sudo ip link add dev can0 type vcan bitrate 1000000
sudo ip link set up can0
```

end
```bash
sudo ip link delete can0
```
