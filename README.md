# SuperCamera Windows Driver

> **Original Linux Driver:** This project is a Windows port of [JDhaworth's SuperCamera ROS Linux driver](https://github.com/Jdhaworth/supercamera_ros)
> 
> **Current Status:** Attempt #5 - Windows 10 Driver Implementation

## Overview

USB driver for UseePlus/SuperCamera USB borescope/endoscope cameras on Windows 10.

**Supported cameras:** USB devices with VID:2ce3 PID:3828 (commonly sold as "UseePlus", "Geek Szitman", or generic USB borescope cameras)

## Technical Details

### Protocol

The camera uses a proprietary USB bulk transfer protocol (not UVC):
- USB header: 5 bytes (magic 0xBBAA, camera ID, payload length)
- Camera header: 7 bytes (frame ID, camera number, flags)
- JPEG payload data

### Camera Specifications

- Resolution: 640x480
- Frame rate: ~13 FPS (hardware limited)
- Latency: <100ms
- USB Protocol: Proprietary bulk transfers

### Known Limitations

- Frame drops (~3%): Due to USB stack limitations with this camera's proprietary protocol, approximately 3% of frames may be incomplete during streaming
- Hardware limited to ~13 FPS
- Requires Windows 10 USB driver installation

## Language Composition

- **C** (48.4%): Core driver implementation
- **Python** (40.1%): Configuration and utility scripts
- **Shell** (10.9%): Build and deployment scripts
- **Makefile** (0.6%): Build configuration

## License

MIT License

## Credits

Original Linux implementation by [JDhaworth](https://github.com/Jdhaworth) - [supercamera_ros](https://github.com/Jdhaworth/supercamera_ros)