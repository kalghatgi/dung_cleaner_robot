#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as RosPath # Use alias to avoid name collision
from geometry_msgs.msg import PoseStamped
import numpy as np
from PIL import Image
from skimage import measure
import traceback # For logging exceptions

# --- Fields2Cover Imports ---
# Based on dir(f2c) output for the installed version
try:
    import fields2cover as f2c
    # Core geometry, robot, path types
    from fields2cover import Point, LinearRing, Cell, Field, Robot, Cells
    from fields2cover import VectorPoint, Swath, Swaths, PathState # Path structure
    # Headland generation
    from fields2cover import HG_Const_gen
    # Swath generation (including angle finding)
    from fields2cover import SG_BruteForce, OBJ_SGObjective # Import objective class
    # Route planning (sorting) - Choose one specific algorithm
    from fields2cover import RP_Snake # Or RP_Boustrophedon, RP_Spiral
    # Path planning (turns)
    from fields2cover import PP_PathPlanning
    # Specific turning strategy class
    from fields2cover import PP_DubinsCurves # Or PP_ReedsSheppCurves if needed
    # Objectives Constants (use directly)
    from fields2cover import OBJ_NSwath, OBJ_PathLength # Constants seem usable

except ImportError as e:
    print("\n--- Error importing Fields2Cover components ---")
    print(f"Failed to import: {e}")
    print("Please check your Fields2Cover installation and version.")
    print("Run the following in python3:")
    print("  import fields2cover as f2c")
    print("  print(dir(f2c))")
    print("And compare the output to the names being imported here.")
    print("---------------------------------------------\n")
    raise # Re-raise the exception after printing info

class CoveragePathPlanner(Node):
    def __init__(self):
        """Initializes the ROS 2 node, publisher, parameters, and triggers path generation."""
        super().__init__('coverage_path_planner')
        # Publisher for the coverage path (uses ROS Path message)
        self.publisher_ = self.create_publisher(RosPath, 'coverage_path', 10)

        # Declare and get parameters
        self.declare_parameter('robot_width_meters', 0.5)
        self.robot_width = self.get_parameter('robot_width_meters').get_parameter_value().double_value
        self.get_logger().info(f"Using robot width: {self.robot_width} m")

        # Load map and generate path during initialization
        try:
            self.load_and_generate_from_map()
        except Exception as e:
            # Log initialization errors with traceback
            tb_str = traceback.format_exc()
            self.get_logger().error(f"Failed to initialize path planner: {e}\nTraceback:\n{tb_str}")
            # Optional: shutdown if initialization fails critically
            # rclpy.shutdown()

    def load_and_generate_from_map(self):
        """Loads map metadata, reads the PGM image, extracts the boundary, and generates the coverage path."""
        # Full path to map YAML file - consider making this a parameter
        yaml_path = '/home/suru/Documents/dung_cleaner_robot/ROS/src/suru_v1_0/maps/terrace_map.yaml' # Using hardcoded path for now
        self.get_logger().info(f"Loading map from: {yaml_path}")

        if not os.path.exists(yaml_path):
             self.get_logger().error(f"Map YAML file not found: {yaml_path}")
             raise FileNotFoundError(f"Map YAML file not found: {yaml_path}")

        # Load YAML metadata
        with open(yaml_path, 'r') as f:
            map_metadata = yaml.safe_load(f)

        # Derive .pgm file path from .yaml
        pgm_path = os.path.join(os.path.dirname(yaml_path), map_metadata['image'])
        self.get_logger().info(f"Loading PGM image from: {pgm_path}")

        if not os.path.exists(pgm_path):
             self.get_logger().error(f"Map PGM file not found: {pgm_path}")
             raise FileNotFoundError(f"Map PGM file not found: {pgm_path}")

        # Get map properties
        resolution = float(map_metadata['resolution'])  # meters/pixel
        origin = map_metadata['origin']  # [x, y, yaw]
        self.get_logger().info(f"Map resolution: {resolution}, Origin: {origin}")

        # Load PGM image
        try:
            image = Image.open(pgm_path).convert('L')
            map_data = np.array(image)
        except Exception as e:
            self.get_logger().error(f"Failed to load or process PGM image: {e}")
            raise

        # --- Define Free Space ---
        # Adjust threshold as needed for your map's grayscale values
        # Usually lighter colors = free space. map_data > 200 assumes white/light gray is free.
        free_space_threshold = 200
        free_space = map_data > free_space_threshold
        self.get_logger().info(f"Found {np.sum(free_space)} free space pixels (threshold > {free_space_threshold}).")

        if np.sum(free_space) == 0:
            self.get_logger().error("No free space found in the map based on the threshold.")
            raise ValueError("No free space found in map.")

        # --- Extract Boundary Polygon ---
        self.get_logger().info("Extracting boundary contours...")
        contours = self.extract_outer_boundary(free_space, resolution, origin)
        self.get_logger().info(f"Extracted boundary with {len(contours)} points.")

        # --- Create Fields2Cover Geometry ---
        points_list = [f2c.Point(x, y) for x, y in contours]
        if not points_list:
             self.get_logger().error("Contour extraction resulted in zero points.")
             raise ValueError("Contour extraction resulted in zero points.")

        try:
            # Create F2C Point vector and LinearRing
            point_vector = f2c.VectorPoint(points_list)
            ring = f2c.LinearRing(point_vector)
        except Exception as e:
             self.get_logger().error(f"Error creating LinearRing: {e}", exc_info=True)
             raise

        # Create Cell, Cells collection, and Field
        cell = f2c.Cell(ring)
        cells_collection = f2c.Cells(cell) # Constructor takes single Cell
        field = f2c.Field(cells_collection) # Constructor takes Cells collection
        # field.setEPSG(0) # Removed - Method does not exist in this version

        # --- Setup Planning Components ---
        robot = f2c.Robot(self.robot_width, self.robot_width)

        # Generate (empty) headlands - Call might be needed for API consistency
        const_headland = f2c.HG_Const_gen()
        no_headland = const_headland.generateHeadlands(field.getField(), 0.0)
        # field.setField(...) # Removed - Subtraction '-' not defined for Cells

        # Instantiate Turning Strategy (e.g., Dubins)
        self.get_logger().info("Using Dubins curves for turns.")
        try:
            turn_planner = f2c.PP_DubinsCurves() # Constructor takes no args
        except Exception as e_turn:
             tb_str_turn = traceback.format_exc()
             self.get_logger().error(f"Error initializing turn planner (PP_DubinsCurves): {e_turn}\nTraceback:\n{tb_str_turn}")
             return

        # --- Generate Swaths ---
        self.get_logger().info("Generating swaths using SG_BruteForce...")
        sg_gen = f2c.SG_BruteForce()
        try:
             # Create objective object (no args needed for constructor)
             sg_objective_object = f2c.OBJ_SGObjective()
             # Generate swaths using objective object, width, and the single Cell
             swaths = sg_gen.generateBestSwaths(sg_objective_object, robot.op_width, cell)
        except Exception as e:
             tb_str_swath = traceback.format_exc()
             self.get_logger().error(f"Error calling SG_BruteForce.generateBestSwaths: {e}\nTraceback:\n{tb_str_swath}")
             return

        self.get_logger().info(f"Generated {swaths.size()} swaths.")
        if swaths.size() == 0:
             self.get_logger().error("Swath generation resulted in zero swaths.")
             return

        # --- Prepare for Path Planning ---
        # Sorter object (e.g., Snake) - not explicitly passed to searchBestPath in this version
        self.get_logger().info("Note: RP_Snake sorter object created but not passed explicitly to searchBestPath.")
        sorter = f2c.RP_Snake()

        # Path Planner (likely needs robot info)
        self.get_logger().info("Planning path with turns using PP_PathPlanning...")
        try:
            path_planner = f2c.PP_PathPlanning(robot) # Try initializing WITH robot first
        except TypeError:
            self.get_logger().warning("PP_PathPlanning constructor might not accept robot. Trying PP_PathPlanning().")
            try:
                 path_planner = f2c.PP_PathPlanning()
            except Exception as e_inner:
                 tb_str_pp_init = traceback.format_exc()
                 self.get_logger().error(f"Could not initialize PP_PathPlanning: {e_inner}\nTraceback:\n{tb_str_pp_init}")
                 return
        except Exception as e:
             tb_str_pp_init = traceback.format_exc()
             self.get_logger().error(f"Could not initialize PP_PathPlanning: {e}\nTraceback:\n{tb_str_pp_init}")
             return

        # --- Generate Final Path ---
        try:
            # Call searchBestPath with robot, swaths, turn_planner
            self.get_logger().info(f"Attempting to call searchBestPath on {type(path_planner)} with robot, swaths, turn_planner...")
            path = path_planner.searchBestPath(robot, swaths, turn_planner)
        except AttributeError:
             tb_str_pp_plan = traceback.format_exc()
             self.get_logger().error(f"Method searchBestPath not found.\nTraceback:\n{tb_str_pp_plan}")
             return
        except Exception as e:
             # Catch other errors (e.g., incorrect types passed)
             tb_str_pp_plan = traceback.format_exc()
             self.get_logger().error(f"Error calling PP_PathPlanning planning method (searchBestPath): {e}\nTraceback:\n{tb_str_pp_plan}")
             return

        self.get_logger().info("Path planning complete.")

        # --- Publish Path ---
        self.publish_path(path)

    def extract_outer_boundary(self, free_space_mask, resolution, origin):
        """
        Extracts the largest outer boundary from a boolean free space map.

        Args:
            free_space_mask: 2D numpy array where True indicates free space.
            resolution: Map resolution in meters/pixel.
            origin: List/tuple [x, y, yaw] of the map origin in world coordinates.

        Returns:
            List of (x, y) tuples representing the boundary in world coordinates.
        """
        # Pad the mask slightly to ensure closed contours at the edges
        padded_mask = np.pad(free_space_mask, pad_width=1, mode='constant', constant_values=0)

        # Find contours using skimage
        contours = measure.find_contours(padded_mask, 0.5) # 0.5 threshold works well for boolean masks
        if not contours:
            self.get_logger().warning("No contours found in free space mask.")
            return []

        # Choose the largest contour (longest perimeter)
        contour = max(contours, key=len)

        # Convert pixel coordinates (origin top-left) to world coordinates (origin bottom-left)
        world_coords = []
        map_height = free_space_mask.shape[0] # Original height
        for point in contour:
            # Adjust for padding: subtract 1 from pixel coords (row, col)
            pixel_y, pixel_x = point[0] - 1, point[1] - 1

            # Convert pixel to world
            # World X = origin_x + pixel_x * resolution
            # World Y = origin_y + (map_height - 1 - pixel_y) * resolution (y points up in world, down in image)
            wx = origin[0] + pixel_x * resolution
            wy = origin[1] + (map_height - 1 - pixel_y) * resolution
            world_coords.append((wx, wy))

        # Ensure the polygon is closed (first and last points are the same)
        if world_coords and world_coords[0] != world_coords[-1]:
             world_coords.append(world_coords[0])

        self.get_logger().info(f"Selected contour length: {len(contour)} pixels. World coords: {len(world_coords)} points.")
        return world_coords

    def publish_path(self, fields2cover_path: f2c.Path):
        """
        Converts a Fields2Cover Path object to a ROS nav_msgs/Path message and publishes it.

        Args:
            fields2cover_path: The f2c.Path object generated by the planner.
        """
        # Create the ROS Path message
        msg = RosPath()
        msg.header.frame_id = 'map' # Ensure this matches your RViz fixed frame
        msg.header.stamp = self.get_clock().now().to_msg()

        pose_count = 0
        if isinstance(fields2cover_path, f2c.Path):
             self.get_logger().info(f"Fields2Cover Path has {fields2cover_path.size()} states.")

             try:
                 # Access the path states using the '.states' attribute
                 path_states = fields2cover_path.states
                 self.get_logger().info(f"Accessed .states attribute (type: {type(path_states)}). Attempting to iterate...")

                 # Iterate over the retrieved states collection
                 for state in path_states:
                     # Check if the item is the expected PathState type
                     if not isinstance(state, f2c.PathState):
                         self.get_logger().warning(f"Item in path.states is not PathState: {type(state)}. Skipping.")
                         continue

                     # Extract point and angle
                     pt = state.point
                     angle_rad = state.angle

                     # Create a PoseStamped message
                     pose = PoseStamped()
                     pose.header = msg.header # Use the same header as the Path msg
                     pose.pose.position.x = pt.getX()
                     pose.pose.position.y = pt.getY()
                     pose.pose.position.z = pt.getZ() if hasattr(pt, 'getZ') else 0.0 # Handle 2D/3D points

                     # Validate angle before converting to quaternion
                     if angle_rad is None or not np.isfinite(angle_rad):
                         self.get_logger().warning(f"Invalid angle ({angle_rad}) in PathState at index {pose_count}. Using default orientation (0 rad).")
                         angle_rad = 0.0 # Default to facing forward along X

                     # Convert yaw angle (rotation around Z) to quaternion
                     cy = np.cos(angle_rad * 0.5)
                     sy = np.sin(angle_rad * 0.5)
                     pose.pose.orientation.w = cy
                     pose.pose.orientation.x = 0.0
                     pose.pose.orientation.y = 0.0
                     pose.pose.orientation.z = sy

                     msg.poses.append(pose)
                     pose_count += 1

             except AttributeError:
                 # Fallback if .states attribute somehow doesn't exist
                 self.get_logger().error("'states' attribute not found on f2c.Path object.", exc_info=True)
                 return
             except TypeError as iter_err:
                 # Handle case where .states exists but is not iterable
                 self.get_logger().error(f"Path.states object (type: {type(path_states)}) is not iterable: {iter_err}", exc_info=True)
                 return
             except Exception as e:
                 # Catch other potential errors during processing
                 self.get_logger().error(f"Error processing path states: {e}", exc_info=True)
                 return

        else:
            # Log error if the input is not the expected f2c.Path type
            self.get_logger().error(f"Unexpected path type received for publishing: {type(fields2cover_path)}. Expected f2c.Path.")
            return

        # Publish the path if poses were generated
        if pose_count > 0:
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published ROS path with {pose_count} poses.")
        else:
            self.get_logger().warning("Generated path contains no poses to publish (or failed during processing).")


def main(args=None):
    """Main function to initialize ROS, create and spin the node, and handle shutdown."""
    rclpy.init(args=args)
    planner = None # Define planner outside try block for finally clause
    try:
        planner = CoveragePathPlanner()
        rclpy.spin(planner)
    except KeyboardInterrupt:
        print("Planner node stopped cleanly by user (KeyboardInterrupt).")
    except Exception as e:
        # Log critical errors during node execution
        print(f"Critical error running planner node: {e}")
        tb_str_main = traceback.format_exc()
        print(f"Traceback:\n{tb_str_main}")
    finally:
        # Ensure proper cleanup and shutdown
        if planner is not None and isinstance(planner, Node) and rclpy.ok():
             # Check if the node was actually spun up before trying to destroy
             if hasattr(planner, '_executor') and planner._executor:
                 if not planner._executor.shutdown_requested:
                     # This assumes a SingleThreadedExecutor or similar
                     planner._executor.shutdown()
             planner.destroy_node()
             print("Planner node destroyed.")
        if rclpy.ok():
            rclpy.shutdown()
            print("rclpy shutdown complete.")

if __name__ == '__main__':
    main()