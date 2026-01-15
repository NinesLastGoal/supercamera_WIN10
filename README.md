# SuperCamera ROS

ROS2 driver for UseePlus/SuperCamera USB borescope/endoscope cameras.

**Supported cameras:** USB devices with VID:2ce3 PID:3828 (commonly sold as "UseePlus", "Geek Szitman", or generic USB borescope cameras)

**Tested on:** Ubuntu 18.04, 20.04, 22.04 with ROS2 Humble/Iron

## Quick Start

### 1. Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/supercamera_ros.git
cd ~/ros2_ws
colcon build --packages-select supercamera_ros
source install/setup.bash
```

### 2. Install the Kernel Driver

```bash
cd ~/ros2_ws/src/supercamera_ros/scripts
sudo ./install_driver.sh
```

This installs a kernel module that creates `/dev/supercamera` when the camera is plugged in.

### 3. Run the Publisher

```bash
# Make sure camera is plugged in
ros2 run supercamera_ros publisher
```

View the images:
```bash
ros2 run rqt_image_view rqt_image_view /supercamera/image_raw
```

## Usage

### Basic Usage

```bash
ros2 run supercamera_ros publisher
```

### With Custom Topic

```bash
ros2 run supercamera_ros publisher --ros-args -p topic:=/my/camera/image
```

### Launch File

```bash
ros2 launch supercamera_ros supercamera.launch.py
ros2 launch supercamera_ros supercamera.launch.py topic:=/endoscope/image_raw
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device` | `/dev/supercamera` | Path to camera device |
| `topic` | `/supercamera/image_raw` | Topic to publish images |
| `frame_id` | `supercamera_link` | TF frame ID |
| `queue_size` | `10` | Publisher queue size |

## Troubleshooting

### Camera not detected

1. Check if camera is plugged in: `lsusb | grep 2ce3`
2. Check if driver is loaded: `lsmod | grep supercamera`
3. Check if device exists: `ls -la /dev/supercamera`
4. Reinstall driver: `sudo ./install_driver.sh`

### Permission denied

```bash
sudo chmod 666 /dev/supercamera
```

### Frame drops (~3%)

Due to Linux USB stack limitations with this camera's proprietary protocol, approximately 3% of frames may be incomplete. The driver automatically repeats the last good frame to maintain smooth video.

## Technical Details

### Protocol

The camera uses a proprietary USB bulk transfer protocol (not UVC):
- USB header: 5 bytes (magic 0xBBAA, camera ID, payload length)
- Camera header: 7 bytes (frame ID, camera number, flags)
- JPEG payload data

### Performance

- Resolution: 640x480
- Frame rate: ~13 FPS (hardware limited)
- Latency: <100ms

## License

MIT License

## Contributing

Pull requests welcome! Please test on Ubuntu 18.04/20.04/22.04 before submitting.
