import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

class NavigationNode(Node):
    def _init_(self):
        super().__init__('navigation_node')
        self.marker_detected = False
        self.goals = [
            {'point': 'power_unit', 'x': -128223, 'y': -651900},
            #{'point': 'energy_cooling_station', 'x': 0, 'y': 0},
            #{'point': 'communication_unit', 'x': 0, 'y': 20},
            #{'point': 'telescope', 'x': 20, 'y': 0},
            #{'point': 'airlock', 'x': 0, 'y': -20}
        ]
        self.current_goal_index = 0
        self.current_point = self.goals[self.current_goal_index]['point']
        self.costmap_received = False
        self.navigate_to_next_goal()

    def odometry_callback(self, msg):
        if self.marker_detected:
            self.marker_detected = False
            
            self.navigate_to_next_goal()
        
    def costmap_callback(self, msg):
        self.costmap_received = True
        
    def navigate_to_next_goal(self):
        goal = self.goals[self.current_goal_index]
        next_x = goal['x']
        next_y = goal['y']

        
        print('detect')
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = next_x
        goal_pose.pose.position.y = next_y
        goal_pose.pose.orientation.w = 1.0
        self.navigator.goToPose(goal_pose)
        self.get_logger().info(f"Navigating to {goal['point']}")
        self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
        self.current_point = self.goals[self.current_goal_index]['point']
        
        self.get_logger().info("Waiting for costmap...")

    def detect_marker(self):
        
        self.marker_detected = True

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode('navigation_node')
    
    try:
        while rclpy.ok():
            rclpy.spin_once(navigation_node)
            if not navigation_node.marker_detected:
                navigation_node.detect_marker()
    except KeyboardInterrupt:
        pass

    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()