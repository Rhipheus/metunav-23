#!/usr/bin/env python3
import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped, Quaternion
from geographic_msgs.msg import GeoPoint
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import math

def geo_to_cartesian_f(tf_buffer,latitude, longitude, altitude):
    # Earth radius in meters
    R = 6371000.0
    # Convert latitude and longitude to radians
    lat_rad = math.radians(latitude)
    lon_rad = math.radians(longitude)

    # Convert geodetic coordinates to Cartesian coordinates
    x = R * math.cos(lat_rad) * math.cos(lon_rad)
    y = R * math.cos(lat_rad) * math.sin(lon_rad)
    z = R * math.sin(lat_rad)

    # Create a GeoPoint message
    geo_point = GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude)

    # Create a PoseStamped message
    pose_stamped_gps = PoseStamped()

    pose_stamped_gps.header.stamp = rclpy.time.Time().to_msg()#rclpy.clock.Clock().now().to_msg()
    pose_stamped_gps.header.frame_id = 'odom'  # Set your frame ID as needed
    # Convert GeoPoint to PoseStamped using tf2
    target_frame = 'odom'  # Set your target frame ID as needed

    try:
        

        transform_gps = tf_buffer.lookup_transform(
            target_frame='odom',
            source_frame='utm',  # Assuming the initial frame is 'map'
            time=rclpy.time.Time().to_msg(),
            timeout=rclpy.time.Duration(seconds=1.0)
        )

        pose_stamped_gps.pose.position.x = x
        pose_stamped_gps.pose.position.y = y
        pose_stamped_gps.pose.position.z = z

        pose_stamped_gps = do_transform_pose(pose_stamped_gps.pose, transform_gps)
        print(f"pose={pose_stamped_gps}")
        
    except Exception as e:
        print(f"Error transforming coordinates: {e}")

    return pose_stamped_gps,