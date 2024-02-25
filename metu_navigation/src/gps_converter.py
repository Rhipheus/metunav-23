#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import NavSatFix
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from utils import geo_to_cartesian

class GPSMapTransform(Node):
    
    def __init__(self):
        super().__init__("gps_transform_node")
        self.get_logger().info("gps_transform_node")
        self.tf_buffer = tf2_ros.Buffer()
        print("initate tf_buffer")
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)
        gps_sub = self.create_subscription(
        NavSatFix, 'gps/fix', self.gps_callback, 10
    )
        print("initate subs")

    """
    def transform_gps_to_local(self,gps_pose):
        try:
            # Lookup the transform between the GPS frame and the local map frame
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=gps_pose.header.frame_id,
                time=gps_pose.header.stamp,
                timeout=rclpy.time.Duration(seconds=1.0)
            )

            # Convert PoseStamped to TransformStamped
            gps_transform = TransformStamped()
            gps_transform.transform.translation = gps_pose.pose.position
            gps_transform.transform.rotation = gps_pose.pose.orientation
            gps_transform.header = gps_pose.header

            # Perform the coordinate transformation
            local_pose = do_transform_pose(gps_transform, transform_stamped)

            return local_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(f"Error transforming GPS to local coordinates: {e}")
            return None
    """

    def gps_callback(self,msg):
        # Transform GPS coordinates to local map coordinates
        print("gps_callback")
        geo_to_cartesian.geo_to_cartesian_f(self.tf_buffer,msg.latitude,msg.longitude,0.0)
        print("geo_to_cartesian_f")
        #local_pose = self.transform_gps_to_local(msg.pose)

        #if local_pose:
        #    # Process the transformed pose as needed
        #   print(f"Local Map Coordinates: {local_pose.pose.position.x}, {local_pose.pose.position.y}")

def main():
    rclpy.init()

    node = rclpy.create_node("gps_transform_node")
    
    node=GPSMapTransform()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()