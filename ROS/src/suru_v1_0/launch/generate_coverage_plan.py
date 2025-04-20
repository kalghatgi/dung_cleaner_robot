import rclpy
from rclpy.node import Node
import geojson
import fields2cover as f2c
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CoveragePathPlanner(Node):
    def __init__(self):
        super().__init__('coverage_path_planner')
        self.publisher_ = self.create_publisher(Path, 'coverage_path', 10)
        self.load_geojson('/mnt/data/test_output.geojson')  # Update with actual path

    def load_geojson(self, geojson_file):
        with open(geojson_file, 'r') as f:
            data = geojson.load(f)
        
        # Extract first polygon
        polygon_coords = data['features'][0]['geometry']['coordinates'][0]
        
        # Convert to Fields2Cover format
        field = f2c.Field()
        field.boundary = f2c.Geometry(polygon_coords)
        
        self.generate_coverage_path(field)
    
    def generate_coverage_path(self, field):
        # Generate coverage paths using Fields2Cover
        coverage_generator = f2c.CoverageGenerator()
        paths = coverage_generator.generate(field)
        
        # Publish the coverage path to ROS 2
        self.publish_path(paths)
    
    def publish_path(self, paths):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        
        for path in paths.get_lines():
            for pt in path.points:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = pt.x
                pose.pose.position.y = pt.y
                path_msg.poses.append(pose)
        
        self.publisher_.publish(path_msg)
        self.get_logger().info('Published coverage path')
        
        # Optional: Visualize
        self.visualize_path(paths)
    
    def visualize_path(self, paths):
        fig, ax = plt.subplots()
        for path in paths.get_lines():
            x_vals = [pt.x for pt in path.points]
            y_vals = [pt.y for pt in path.points]
            ax.plot(x_vals, y_vals, marker='o')
        plt.show()

def main():
    rclpy.init()
    planner = CoveragePathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
