#!/usr/bin/env python3
"""
SuperCamera ROS2 Publisher Node

Publishes images from UseePlus/SuperCamera USB borescope cameras to ROS2 topics.
Supports cameras with VID:2ce3 PID:3828.

Usage:
    ros2 run supercamera_ros publisher
    ros2 run supercamera_ros publisher --ros-args -p device:=/dev/supercamera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import os
import threading
import queue
import time


class SuperCameraPublisher(Node):
    """ROS2 node that publishes images from SuperCamera USB device."""
    
    def __init__(self):
        super().__init__('supercamera_publisher')
        
        # Declare parameters
        self.declare_parameter('device', '/dev/supercamera')
        self.declare_parameter('topic', '/supercamera/image_raw')
        self.declare_parameter('frame_id', 'supercamera_link')
        self.declare_parameter('queue_size', 10)
        
        # Get parameters
        self.device_path = self.get_parameter('device').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        queue_size = self.get_parameter('queue_size').value
        
        # Publisher
        self.pub = self.create_publisher(Image, topic, queue_size)
        self.bridge = CvBridge()
        
        # State
        self.running = True
        self.fd = None
        self.frame_count = 0
        
        # Open device
        if not self._open_device():
            self.get_logger().error(f'Failed to open {self.device_path}')
            self.get_logger().error('Make sure the driver is installed: run install_driver.sh')
            return
        
        # Start capture thread
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f'SuperCamera started: {self.device_path} -> {topic}')
    
    def _open_device(self) -> bool:
        """Open the character device."""
        if not os.path.exists(self.device_path):
            self.get_logger().error(f'Device not found: {self.device_path}')
            self.get_logger().error('Is the camera plugged in?')
            return False
        
        try:
            self.fd = os.open(self.device_path, os.O_RDONLY)
            return True
        except Exception as e:
            self.get_logger().error(f'Cannot open device: {e}')
            return False
    
    def _capture_loop(self):
        """Main capture loop - reads USB data and publishes frames."""
        # Queue for JPEG frames (reader -> publisher)
        frame_queue = queue.Queue(maxsize=30)
        
        # Start reader thread
        reader = threading.Thread(
            target=self._reader_thread,
            args=(frame_queue,),
            daemon=True
        )
        reader.start()
        
        # Publish frames
        frames_published = 0
        frames_dropped = 0
        last_good_frame = None
        start_time = time.time()
        
        while self.running:
            try:
                result = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            
            jpeg_data, stats = result
            
            # Handle dropped frames by repeating last good frame
            if jpeg_data is None:
                frames_dropped += 1
                if last_good_frame is not None:
                    self._publish_frame(last_good_frame)
                    frames_published += 1
                continue
            
            # Decode JPEG
            frame = cv.imdecode(
                np.frombuffer(jpeg_data, dtype=np.uint8),
                cv.IMREAD_COLOR
            )
            
            if frame is not None:
                last_good_frame = frame
                self._publish_frame(frame)
                frames_published += 1
            else:
                # Decode failed - repeat last good frame
                frames_dropped += 1
                if last_good_frame is not None:
                    self._publish_frame(last_good_frame)
                    frames_published += 1
            
            # Log every 100 frames
            if frames_published % 100 == 0 and frames_published > 0:
                elapsed = time.time() - start_time
                fps = frames_published / elapsed
                drop_pct = 100 * frames_dropped / (frames_published + frames_dropped) if frames_dropped > 0 else 0
                self.get_logger().info(
                    f'Published {frames_published} frames @ {fps:.1f} FPS ({drop_pct:.1f}% dropped)'
                )
        
        self.get_logger().info('Capture loop ended')
    
    def _reader_thread(self, frame_queue):
        """
        USB reader thread - parses packets and extracts JPEG frames.
        """
        USB_HDR = 5
        CAM_HDR = 7
        TOTAL_HDR = USB_HDR + CAM_HDR
        READ_SIZE = 65536
        JPEG_SOI = b'\xff\xd8'
        JPEG_EOI = b'\xff\xd9'
        
        raw_buffer = bytearray()
        jpeg_buffer = bytearray()
        frames_good = 0
        frames_bad = 0
        
        while self.running:
            try:
                chunk = os.read(self.fd, READ_SIZE)
                if not chunk:
                    continue
                raw_buffer.extend(chunk)
                
                # Process complete packets
                while len(raw_buffer) >= TOTAL_HDR:
                    # Check for valid packet header (magic: AA BB)
                    if raw_buffer[0] != 0xAA or raw_buffer[1] != 0xBB:
                        # Lost sync - find next header
                        idx = raw_buffer.find(b'\xaa\xbb')
                        if idx > 0:
                            raw_buffer = raw_buffer[idx:]
                        elif idx < 0:
                            raw_buffer.clear()
                        continue
                    
                    # Parse length from header
                    payload_len = raw_buffer[3] | (raw_buffer[4] << 8)
                    packet_size = USB_HDR + payload_len
                    
                    # Sanity check length
                    if payload_len < CAM_HDR or payload_len > 2000:
                        raw_buffer = raw_buffer[1:]
                        continue
                    
                    # Wait for complete packet
                    if len(raw_buffer) < packet_size:
                        break
                    
                    # Extract JPEG payload (after both headers)
                    jpeg_payload = raw_buffer[TOTAL_HDR:packet_size]
                    
                    # Check if this starts a new JPEG frame
                    if len(jpeg_payload) >= 2 and jpeg_payload[:2] == JPEG_SOI:
                        # New frame - check if previous frame is complete
                        if len(jpeg_buffer) > 1000:
                            eoi_pos = jpeg_buffer.rfind(JPEG_EOI)
                            if eoi_pos > 0:
                                jpeg_data = bytes(jpeg_buffer[:eoi_pos + 2])
                                frames_good += 1
                                try:
                                    frame_queue.put_nowait(
                                        (jpeg_data, (frames_good, frames_bad))
                                    )
                                except queue.Full:
                                    pass
                            else:
                                frames_bad += 1
                                try:
                                    frame_queue.put_nowait((None, (frames_good, frames_bad)))
                                except queue.Full:
                                    pass
                        
                        # Start new frame
                        jpeg_buffer = bytearray(jpeg_payload)
                    else:
                        # Continue accumulating
                        jpeg_buffer.extend(jpeg_payload)
                    
                    # Advance past this packet
                    raw_buffer = raw_buffer[packet_size:]
                    
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f'Reader error: {e}')
    
    def _publish_frame(self, frame):
        """Publish a frame to ROS2."""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)
            self.frame_count += 1
        except Exception as e:
            self.get_logger().warn(f'Publish error: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.running = False
        if self.fd is not None:
            try:
                os.close(self.fd)
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SuperCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
