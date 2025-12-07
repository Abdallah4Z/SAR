#!/usr/bin/env python3
"""
YOLO Object Detection with AirSim and ROS2 Visualization
========================================================
This script demonstrates:
1. YOLO object detection on drone camera feed
2. Publishing detections to ROS2 topics
3. Visualization in RViz2 with bounding boxes and labels
4. Integration with AirSim swarm

Requirements:
    pip install ultralytics opencv-python
    
Note: This uses YOLOv8 from ultralytics. First run will download the model.
"""

import airsim
import cv2
import numpy as np
import time
from threading import Thread, Lock
import sys

# Try importing YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    print("⚠️  YOLO not available. Install with: pip install ultralytics")
    YOLO_AVAILABLE = False

# Try importing ROS2
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    print("⚠️  ROS2 not available. Install cv_bridge: sudo apt install ros-humble-cv-bridge")
    ROS2_AVAILABLE = False


class DetectionResult:
    """Container for detection results"""
    def __init__(self, class_name, confidence, bbox, world_position=None):
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # (x1, y1, x2, y2)
        self.world_position = world_position
        self.timestamp = time.time()


class YOLODetectorROS2(Node):
    """ROS2 node for YOLO detection and visualization"""
    
    def __init__(self, drone_name="Drone0", camera_name="0"):
        super().__init__(f'yolo_detector_{drone_name}')
        
        self.drone_name = drone_name
        self.camera_name = camera_name
        
        # Initialize AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        # Initialize YOLO
        self.model = None
        if YOLO_AVAILABLE:
            try:
                self.get_logger().info("📥 Loading YOLOv8 model...")
                self.model = YOLO('yolov8n.pt')  # nano model for speed
                self.get_logger().info("✅ YOLO model loaded successfully")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to load YOLO: {e}")
                self.model = None
        
        # ROS2 publishers
        self.image_pub = self.create_publisher(
            Image, 
            f'/{drone_name}/camera/image_detections', 
            10
        )
        self.detection_marker_pub = self.create_publisher(
            MarkerArray, 
            f'/{drone_name}/detections_3d', 
            10
        )
        self.detection_text_pub = self.create_publisher(
            String, 
            f'/{drone_name}/detection_events', 
            10
        )
        
        # CV Bridge for ROS2 image conversion
        self.bridge = CvBridge()
        
        # Detection tracking
        self.detections = []
        self.detection_lock = Lock()
        
        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz
        
        # Stats
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = time.time()
        self.fps = 0
        
        self.get_logger().info(f'🎯 YOLO Detection Node Started for {drone_name}')
        self.get_logger().info(f'   Camera: {camera_name}')
        self.get_logger().info(f'   Publishing to:')
        self.get_logger().info(f'     - /{drone_name}/camera/image_detections')
        self.get_logger().info(f'     - /{drone_name}/detections_3d')
        self.get_logger().info(f'     - /{drone_name}/detection_events')
    
    def process_frame(self):
        """Process camera frame and run detection"""
        try:
            # Get image from AirSim
            responses = self.client.simGetImages([
                airsim.ImageRequest(self.camera_name, airsim.ImageType.Scene, False, False)
            ], vehicle_name=self.drone_name)
            
            if not responses or len(responses[0].image_data_uint8) == 0:
                return
            
            response = responses[0]
            
            # Convert to OpenCV format
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)
            
            # Run YOLO detection
            if self.model:
                detections = self._run_detection(img_rgb)
                
                # Draw detections
                img_annotated = self._draw_detections(img_rgb.copy(), detections)
                
                # Publish annotated image
                self._publish_image(img_annotated)
                
                # Publish 3D markers
                if detections:
                    self._publish_3d_markers(detections)
                
                # Update stats
                self.frame_count += 1
                if time.time() - self.last_fps_time > 1.0:
                    self.fps = self.frame_count / (time.time() - self.last_fps_time)
                    self.frame_count = 0
                    self.last_fps_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
    
    def _run_detection(self, image):
        """Run YOLO detection on image"""
        results = self.model(image, verbose=False)
        
        detections = []
        
        for result in results:
            boxes = result.boxes
            
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Get class and confidence
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.model.names[class_id]
                
                # Filter low confidence
                if confidence < 0.5:
                    continue
                
                # Estimate 3D position (simplified)
                world_pos = self._estimate_3d_position(x1, y1, x2, y2, image.shape)
                
                detection = DetectionResult(
                    class_name=class_name,
                    confidence=confidence,
                    bbox=(int(x1), int(y1), int(x2), int(y2)),
                    world_position=world_pos
                )
                
                detections.append(detection)
                
                # Publish detection event for important objects
                if class_name in ['person', 'car', 'truck', 'fire hydrant', 'stop sign']:
                    self._publish_detection_event(detection)
        
        with self.detection_lock:
            self.detections = detections
            self.detection_count = len(detections)
        
        return detections
    
    def _estimate_3d_position(self, x1, y1, x2, y2, image_shape):
        """Estimate 3D world position of detection (simplified)"""
        try:
            # Get drone state
            state = self.client.getMultirotorState(vehicle_name=self.drone_name)
            drone_pos = state.kinematics_estimated.position
            drone_orient = state.kinematics_estimated.orientation
            
            # Simple projection: assume object is at certain distance
            # In real application, use depth camera or LIDAR
            estimated_distance = 10.0  # meters
            
            # Center of bounding box
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # Normalize to [-1, 1]
            norm_x = (center_x / image_shape[1]) * 2 - 1
            norm_y = (center_y / image_shape[0]) * 2 - 1
            
            # Simple forward projection
            # This is a rough estimate - proper implementation needs camera intrinsics
            offset_x = norm_x * estimated_distance * 0.5
            offset_y = norm_y * estimated_distance * 0.5
            
            world_x = drone_pos.x_val + estimated_distance
            world_y = drone_pos.y_val + offset_x
            world_z = drone_pos.z_val + offset_y
            
            return (world_x, world_y, world_z)
            
        except Exception as e:
            return None
    
    def _draw_detections(self, image, detections):
        """Draw bounding boxes and labels on image"""
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            
            # Color based on class (different colors for different object types)
            color_map = {
                'person': (0, 255, 0),
                'car': (255, 0, 0),
                'truck': (255, 0, 255),
                'bicycle': (0, 255, 255),
                'motorcycle': (255, 255, 0),
            }
            color = color_map.get(det.class_name, (0, 255, 0))
            
            # Draw bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{det.class_name} {det.confidence:.2f}"
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            
            # Background for text
            cv2.rectangle(image, (x1, y1 - label_h - 10), (x1 + label_w, y1), color, -1)
            cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Draw FPS and detection count
        info_text = f"FPS: {self.fps:.1f} | Detections: {len(detections)}"
        cv2.putText(image, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return image
    
    def _publish_image(self, image):
        """Publish annotated image to ROS2"""
        try:
            # Convert BGR to RGB (OpenCV uses BGR)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Convert to ROS2 message
            msg = self.bridge.cv2_to_imgmsg(image_rgb, encoding="rgb8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{self.drone_name}_camera"
            
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")
    
    def _publish_3d_markers(self, detections):
        """Publish 3D markers for detections in RViz2"""
        marker_array = MarkerArray()
        
        for i, det in enumerate(detections):
            if not det.world_position:
                continue
            
            # Create marker
            marker = Marker()
            marker.header.frame_id = "world_ned"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position
            x, y, z = det.world_position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            
            # Size
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0
            
            # Color based on object type
            if det.class_name == 'person':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif det.class_name in ['car', 'truck']:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 0.7
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "detection_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 1.5
            
            text_marker.scale.z = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{det.class_name}\n{det.confidence:.0%}"
            text_marker.lifetime.sec = 1
            
            marker_array.markers.append(text_marker)
        
        self.detection_marker_pub.publish(marker_array)
    
    def _publish_detection_event(self, detection):
        """Publish detection event"""
        msg = String()
        msg.data = f"[{self.drone_name}] Detected: {detection.class_name} ({detection.confidence:.0%})"
        
        self.detection_text_pub.publish(msg)
        
        self.get_logger().info(f'🎯 Detected: {detection.class_name} ({detection.confidence:.0%})')


def main():
    """Main function"""
    if not YOLO_AVAILABLE:
        print("\n❌ YOLO not available. Please install:")
        print("   pip install ultralytics")
        return
    
    if not ROS2_AVAILABLE:
        print("\n❌ ROS2 not available. Please install:")
        print("   sudo apt install ros-humble-cv-bridge")
        return
    
    # Parse arguments
    drone_name = sys.argv[1] if len(sys.argv) > 1 else "Drone0"
    camera_name = sys.argv[2] if len(sys.argv) > 2 else "0"
    
    print(f"\n{'='*70}")
    print(f"🎯 YOLO OBJECT DETECTION WITH ROS2 VISUALIZATION")
    print(f"{'='*70}")
    print(f"Drone: {drone_name}")
    print(f"Camera: {camera_name}")
    print(f"\nVisualization Topics:")
    print(f"  - /{drone_name}/camera/image_detections (Image)")
    print(f"  - /{drone_name}/detections_3d (3D Markers)")
    print(f"  - /{drone_name}/detection_events (Text)")
    print(f"\nAdd these topics in RViz2 to visualize!")
    print(f"{'='*70}\n")
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    node = YOLODetectorROS2(drone_name=drone_name, camera_name=camera_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
