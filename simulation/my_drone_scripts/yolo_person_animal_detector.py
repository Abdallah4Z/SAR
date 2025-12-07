#!/usr/bin/env python3
"""
YOLO Person and Animal Detection for AirSim Drones
Detects people and animals (COCO classes) and publishes to ROS2
Real-time detection with 3D position estimation
"""

import airsim
import cv2
import numpy as np
import time
import threading
import sys

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String, Header
from cv_bridge import CvBridge

# YOLO import
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    print("⚠️  ultralytics not installed. Install with: pip install ultralytics")
    YOLO_AVAILABLE = False
    sys.exit(1)


class YOLOPersonAnimalDetector(Node):
    def __init__(self, drone_name="Drone0", camera_name="front_center"):
        super().__init__(f'yolo_detector_{drone_name.lower()}')
        
        self.drone_name = drone_name
        self.camera_name = camera_name
        
        # Connect to AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.get_logger().info(f"✅ Connected to AirSim for {drone_name}")
        
        # Load YOLO model (YOLOv8 nano for real-time performance)
        self.get_logger().info("🤖 Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt')  # Nano model for speed
        self.get_logger().info("✅ YOLO model loaded")
        
        # COCO classes for people and animals
        self.target_classes = {
            0: 'person',
            14: 'bird',
            15: 'cat', 
            16: 'dog',
            17: 'horse',
            18: 'sheep',
            19: 'cow',
            20: 'elephant',
            21: 'bear',
            22: 'zebra',
            23: 'giraffe'
        }
        
        # ROS2 publishers
        self.image_pub = self.create_publisher(
            Image, 
            f'/{drone_name}/detection_image', 
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            f'/{drone_name}/detections_3d', 
            10
        )
        self.stats_pub = self.create_publisher(
            String,
            f'/{drone_name}/detection_stats',
            10
        )
        
        self.bridge = CvBridge()
        
        # Detection statistics
        self.detection_counts = {class_name: 0 for class_name in self.target_classes.values()}
        self.total_detections = 0
        self.fps = 0.0
        self.last_time = time.time()
        
        # Thread control
        self.running = True
        self.detection_thread = threading.Thread(target=self.detection_loop)
        self.detection_thread.daemon = True
        
        self.get_logger().info(f"🎯 Target classes: {list(self.target_classes.values())}")
        self.get_logger().info("✅ YOLO Person/Animal Detector initialized")
    
    def get_drone_pose(self):
        """Get current drone position and orientation"""
        try:
            state = self.client.getMultirotorState(vehicle_name=self.drone_name)
            pos = state.kinematics_estimated.position
            orientation = state.kinematics_estimated.orientation
            return pos, orientation
        except Exception as e:
            self.get_logger().error(f"Error getting drone pose: {e}")
            return None, None
    
    def estimate_3d_position(self, bbox_center, drone_pos, distance_estimate=50.0):
        """
        Estimate 3D world position of detected object
        Simple projection based on drone position and camera FOV
        
        Args:
            bbox_center: (x, y) center of bounding box in image
            drone_pos: Current drone position
            distance_estimate: Estimated distance to object (meters)
        """
        try:
            # Camera parameters (from settings.json)
            img_width = 1280
            img_height = 720
            fov_horizontal = 90.0  # degrees
            
            # Normalize bbox center to [-1, 1]
            norm_x = (bbox_center[0] - img_width / 2) / (img_width / 2)
            norm_y = (bbox_center[1] - img_height / 2) / (img_height / 2)
            
            # Calculate angle offsets
            angle_x = norm_x * (fov_horizontal / 2) * np.pi / 180
            angle_y = norm_y * (fov_horizontal / 2) * (img_height / img_width) * np.pi / 180
            
            # Simple projection (assumes drone looking forward and level)
            # In NED coordinates: X=forward, Y=right, Z=down
            offset_x = distance_estimate * np.cos(angle_y)
            offset_y = distance_estimate * np.sin(angle_x)
            offset_z = distance_estimate * np.sin(angle_y)
            
            # World position
            world_x = drone_pos.x_val + offset_x
            world_y = drone_pos.y_val + offset_y
            world_z = drone_pos.z_val + offset_z
            
            return (world_x, world_y, world_z)
            
        except Exception as e:
            self.get_logger().error(f"Error estimating 3D position: {e}")
            return (drone_pos.x_val, drone_pos.y_val, drone_pos.z_val)
    
    def create_detection_marker(self, detection_id, class_name, position, confidence):
        """Create RViz marker for detected object"""
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.drone_name}_detections"
        marker.id = detection_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        
        # Scale based on class
        scale = 1.0 if class_name == 'person' else 0.5
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        
        # Color based on class
        colors = {
            'person': (1.0, 0.8, 0.6, 0.8),      # Skin tone
            'dog': (0.6, 0.4, 0.2, 0.8),          # Brown
            'cat': (1.0, 0.6, 0.0, 0.8),          # Orange
            'horse': (0.4, 0.3, 0.2, 0.8),        # Dark brown
            'cow': (0.9, 0.9, 0.9, 0.8),          # White
            'sheep': (0.95, 0.95, 0.8, 0.8),      # Cream
            'bird': (0.2, 0.5, 0.8, 0.8),         # Blue
            'elephant': (0.5, 0.5, 0.5, 0.8),     # Gray
            'bear': (0.3, 0.2, 0.1, 0.8),         # Dark brown
            'zebra': (0.0, 0.0, 0.0, 0.8),        # Black
            'giraffe': (0.9, 0.7, 0.4, 0.8)       # Tan
        }
        color = colors.get(class_name, (1.0, 1.0, 0.0, 0.8))
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Lifetime
        marker.lifetime.sec = 2
        
        return marker
    
    def create_text_marker(self, detection_id, class_name, position, confidence):
        """Create text label marker above detected object"""
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.drone_name}_labels"
        marker.id = detection_id + 10000  # Offset to avoid ID collision
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position above the object
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] - 2.0  # 2m above (NED: negative is up)
        marker.pose.orientation.w = 1.0
        
        # Text
        marker.text = f"{class_name}\n{confidence:.2f}"
        marker.scale.z = 1.0  # Text size
        
        # Color: white text
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 2
        
        return marker
    
    def publish_detections(self, detections, drone_pos):
        """Publish detection markers to RViz"""
        marker_array = MarkerArray()
        
        for i, det in enumerate(detections):
            class_name = det['class']
            bbox_center = det['bbox_center']
            confidence = det['confidence']
            
            # Estimate 3D position
            position_3d = self.estimate_3d_position(bbox_center, drone_pos)
            
            # Create sphere marker
            sphere_marker = self.create_detection_marker(i, class_name, position_3d, confidence)
            marker_array.markers.append(sphere_marker)
            
            # Create text label
            text_marker = self.create_text_marker(i, class_name, position_3d, confidence)
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def draw_detections(self, frame, detections):
        """Draw bounding boxes and labels on frame"""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            class_name = det['class']
            confidence = det['confidence']
            
            # Color based on class
            colors_bgr = {
                'person': (153, 204, 255),
                'dog': (51, 102, 153),
                'cat': (0, 153, 255),
                'horse': (51, 76, 102),
                'cow': (230, 230, 230),
                'sheep': (204, 242, 242),
                'bird': (204, 127, 51),
                'elephant': (127, 127, 127),
                'bear': (25, 51, 76),
                'zebra': (0, 0, 0),
                'giraffe': (102, 178, 230)
            }
            color = colors_bgr.get(class_name, (0, 255, 255))
            
            # Draw box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            
            # Draw label background
            label = f"{class_name} {confidence:.2f}"
            (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (int(x1), int(y1) - label_height - 10), 
                         (int(x1) + label_width, int(y1)), color, -1)
            
            # Draw label text
            cv2.putText(frame, label, (int(x1), int(y1) - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame
    
    def draw_stats_overlay(self, frame):
        """Draw detection statistics on frame"""
        # Semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (400, 300), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Title
        cv2.putText(frame, f"{self.drone_name} - Detection Stats", (20, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # FPS
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (20, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Total detections
        cv2.putText(frame, f"Total: {self.total_detections}", (20, 95),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Per-class counts
        y_offset = 125
        for class_name, count in sorted(self.detection_counts.items()):
            if count > 0:
                cv2.putText(frame, f"  {class_name}: {count}", (20, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                y_offset += 25
        
        return frame
    
    def detection_loop(self):
        """Main detection loop"""
        self.get_logger().info("🚀 Starting detection loop...")
        
        frame_count = 0
        fps_update_interval = 10
        
        while self.running and rclpy.ok():
            try:
                start_time = time.time()
                
                # Get camera image from AirSim
                responses = self.client.simGetImages([
                    airsim.ImageRequest(self.camera_name, airsim.ImageType.Scene, False, False)
                ], vehicle_name=self.drone_name)
                
                if not responses or len(responses[0].image_data_uint8) == 0:
                    time.sleep(0.1)
                    continue
                
                response = responses[0]
                
                # Convert to OpenCV format
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(response.height, response.width, 3)
                
                # Get drone position
                drone_pos, _ = self.get_drone_pose()
                if drone_pos is None:
                    continue
                
                # Run YOLO detection
                results = self.model(img_rgb, verbose=False, conf=0.4)
                
                # Parse detections
                detections = []
                for result in results:
                    for box in result.boxes:
                        class_id = int(box.cls[0])
                        
                        # Only process target classes (people/animals)
                        if class_id in self.target_classes:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            confidence = float(box.conf[0])
                            class_name = self.target_classes[class_id]
                            
                            bbox_center = ((x1 + x2) / 2, (y1 + y2) / 2)
                            
                            detections.append({
                                'bbox': (x1, y1, x2, y2),
                                'bbox_center': bbox_center,
                                'class': class_name,
                                'confidence': confidence
                            })
                            
                            # Update statistics
                            self.detection_counts[class_name] += 1
                            self.total_detections += 1
                
                # Publish 3D markers to RViz
                if detections:
                    self.publish_detections(detections, drone_pos)
                
                # Draw detections on image
                annotated_frame = self.draw_detections(img_rgb.copy(), detections)
                annotated_frame = self.draw_stats_overlay(annotated_frame)
                
                # Publish annotated image
                img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="rgb8")
                self.image_pub.publish(img_msg)
                
                # Calculate FPS
                frame_count += 1
                if frame_count % fps_update_interval == 0:
                    current_time = time.time()
                    self.fps = fps_update_interval / (current_time - self.last_time)
                    self.last_time = current_time
                    
                    # Publish stats
                    stats_msg = String()
                    stats_msg.data = f"FPS:{self.fps:.1f}|Total:{self.total_detections}"
                    self.stats_pub.publish(stats_msg)
                
                # Optional: Display window (comment out for headless)
                # cv2.imshow(f'{self.drone_name} Detections', cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR))
                # cv2.waitKey(1)
                
            except Exception as e:
                self.get_logger().error(f"Detection loop error: {e}")
                time.sleep(0.1)
    
    def start(self):
        """Start detection"""
        self.detection_thread.start()
        self.get_logger().info("✅ Detection started")
    
    def stop(self):
        """Stop detection"""
        self.running = False
        if self.detection_thread.is_alive():
            self.detection_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
        self.get_logger().info("✅ Detection stopped")


def main(args=None):
    if not YOLO_AVAILABLE:
        print("❌ YOLO not available. Exiting.")
        return
    
    # Parse arguments
    drone_name = sys.argv[1] if len(sys.argv) > 1 else "Drone0"
    camera_name = sys.argv[2] if len(sys.argv) > 2 else "front_center"
    
    print("="*70)
    print(f"  🎯 YOLO PERSON/ANIMAL DETECTOR - {drone_name}")
    print("="*70)
    print(f"Camera: {camera_name}")
    print(f"Model: YOLOv8n (nano)")
    print(f"Target Classes: person, dog, cat, horse, cow, sheep, bird, elephant, bear, zebra, giraffe")
    print("="*70)
    print()
    
    rclpy.init(args=args)
    
    detector = YOLOPersonAnimalDetector(drone_name, camera_name)
    
    try:
        detector.start()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        detector.stop()
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
