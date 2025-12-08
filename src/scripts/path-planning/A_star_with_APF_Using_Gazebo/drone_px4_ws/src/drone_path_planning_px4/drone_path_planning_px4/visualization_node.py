#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        # Publishers
        self.start_marker_pub = self.create_publisher(Marker, '/drone/start_marker', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/drone/goal_marker', 10)
        
        # Timer
        self.marker_timer = self.create_timer(2.0, self.publish_markers)
        
        self.get_logger().info('Visualization Node initialized')
        
    def publish_markers(self):
        """Publish start and goal markers"""
        # Start marker (green)
        start_marker = Marker()
        start_marker.header.frame_id = 'map'
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = 'navigation'
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        
        start_marker.pose.position.x = 0.0
        start_marker.pose.position.y = 0.0
        start_marker.pose.position.z = 5.0
        start_marker.pose.orientation.w = 1.0
        
        start_marker.scale.x = 2.0
        start_marker.scale.y = 2.0
        start_marker.scale.z = 2.0
        
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        
        self.start_marker_pub.publish(start_marker)
        
        # Goal marker (red)
        goal_marker = Marker()
        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = 'navigation'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        
        goal_marker.pose.position.x = 40.0
        goal_marker.pose.position.y = 40.0
        goal_marker.pose.position.z = 5.0
        goal_marker.pose.orientation.w = 1.0
        
        goal_marker.scale.x = 2.0
        goal_marker.scale.y = 2.0
        goal_marker.scale.z = 2.0
        
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        
        self.goal_marker_pub.publish(goal_marker)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
