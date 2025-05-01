# ROS:
HEAD:ROS/src/suru_v1_0/README.md
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-joint-state-publisher ros-humble-robot-localization ros-humble-xacro ros-humble-imu-complementary-filter ros-humble-imu-filter-madgwick ros-humble-rmw-cyclonedds-cpp ros-humble-behaviortree-cpp ros-humble-filters

sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-joint-state-publisher ros-humble-robot-localization ros-humble-xacro ros-humble-imu-complementary-filter ros-humble-imu-filter-madgwick ros-humble-filters ros-humble-rmw-cyclonedds-cpp ros-humble-behaviortree-cpp libboost-all-dev build-essential python3-colcon-common-extensions

# microROS:
source /opt/ros/humble/setup.bash
cd ~
mkdir uros_ws && cd uros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt install python3-rosdep2
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# bashrc:
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
alias getros='source /opt/ros/humble/setup.bash'
alias getesp='. ~/esp/esp-idf/export.sh'
alias getnodes='source ~/dung_cleaner_robot/ROS/install/local_setup.bash'
alias geturos='source ~/uros_ws/install/local_setup.bash'

# ROS nodes
cd dung_cleaner_robot/ROS/src/Fields2Cover
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc);
sudo make install
cd ../../../
getros
colcon build --symlink-install

HEAD:ROS/src/suru_v1_0/README.md

####---- ROS 2 Coverage Path Planner using Fields2Cover ----####

This ROS 2 package implements a simple coverage path planner node. It loads a standard ROS map (`.yaml` and `.pgm`), extracts the outer boundary of the free space, uses the Fields2Cover library to generate a coverage path (swaths and turns), and publishes the resulting path as a `nav_msgs/Path` message for visualization in RViz or use by other nodes.

This project was developed for **ROS 2 Humble** and specifically targets the version of **Fields2Cover available via the standard Ubuntu/Debian package manager (`python3-fields2cover`)** as of early 2025. Significant API differences exist compared to newer versions of Fields2Cover, which were discovered during development (see Troubleshooting Log).

## Prerequisites

*   **ROS 2 Humble Hawksbill:** Including `colcon`, `rclpy`, `nav_msgs`, `geometry_msgs`.
*   **Python 3:** (Typically Python 3.10 for Humble).
*   **Required Python Libraries:**
    *   `numpy`
    *   `PyYAML`
    *   `Pillow` (PIL Fork)
    *   `scikit-image`
*   **Fields2Cover:** The specific version installable via `apt` (or corresponding system package manager).
    ```bash
    sudo apt update
    sudo apt install python3-fields2cover # Adjust if package name differs
    ```
    *Note: Using newer versions of Fields2Cover (e.g., installed via pip or built from source) will likely require significant code changes due to API differences.*

## Installation & Setup

1.  **Create/Navigate to ROS 2 Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/
    ```
2.  **Clone the Repository:** (Assuming you place the `suru_v1_0` package inside the `src` directory)
    ```bash
    git clone <your-repo-url> src/suru_v1_0
    # Or manually copy the suru_v1_0 package into src/
    ```
3.  **Install Dependencies:**
    ```bash
    cd ~/ros2_ws
    sudo apt update # Ensure apt cache is updated
    # Install python dependencies if not already present
    sudo apt install python3-numpy python3-yaml python3-pil python3-skimage python3-fields2cover
    # Install ROS dependencies
    rosdep install --from-paths src --ignore-src -r -y
    ```
4.  **Build the Package:**
    ```bash
    colcon build --packages-select suru_v1_0
    ```
5.  **Source the Workspace:**
    ```bash
    source install/setup.bash
    # Or add this to your ~/.bashrc for persistence
    ```

## Configuration

The node currently uses some hardcoded values and parameters:

1.  **Map File:** The path to the map YAML file is hardcoded inside `coverage_planner_node.py` within the `load_and_generate_from_map` function:
    ```python
    yaml_path = '/home/suru/Documents/dung_cleaner_robot/ROS/src/suru_v1_0/maps/terrace_map.yaml'
    ```
    *Modify this line to point to your desired map file.*
2.  **Robot Width:** This is configured via a ROS parameter. You can set it when running the node:
    ```bash
    ros2 run suru_v1_0 coverage_planner_node.py --ros-args -p robot_width_meters:=0.6
    ```
    *The default value in the code is 0.5 meters.*
3.  **Free Space Threshold:** The PGM image pixel value threshold to determine free space is set in `load_and_generate_from_map`:
    ```python
    free_space_threshold = 200
    free_space = map_data > free_space_threshold
    ```
    *Adjust `free_space_threshold` based on your map's grayscale conventions (typical maps use values near 254 for free space).*

## Usage

1.  **Source your workspace:** `source install/setup.bash`
2.  **Run the node:**
    ```bash
    ros2 run suru_v1_0 coverage_planner_node.py
    # Or with custom robot width:
    # ros2 run suru_v1_0 coverage_planner_node.py --ros-args -p robot_width_meters:=<your_width>
    ```
3.  **Visualize in RViz:**
    *   Launch RViz: `rviz2`
    *   Set the "Fixed Frame" to `map`.
    *   Add a "Path" display.
    *   Set the "Topic" for the Path display to `/coverage_path`.
    *   You should see the generated coverage path overlaid on the map (if you also display the map).

*Note: You might see a `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed` message upon startup. This is a common Fast DDS shared memory transport issue and often does not affect functionality.*

## Node Details

*   **Published Topic:** `/coverage_path` (`nav_msgs/Path`) - The calculated coverage path.
*   **Parameters:** `robot_width_meters` (double, default: 0.5) - The operational width of the robot used for swath generation.
*   **Input:** Reads map data (`.yaml` metadata and `.pgm` image) from the configured file path.

## Troubleshooting Log & Development Notes

This script underwent significant debugging primarily due to API differences and naming conventions in the system-installed version of Fields2Cover compared to newer versions or documentation examples. Key issues encountered and resolved:

1.  **`TypeError: new_LinearRing`:** Initial attempt passed a Python `list` of `f2c.Point` to `f2c.LinearRing`. **Fix:** Converted the list to the required `f2c.VectorPoint` type first.
2.  **`ImportError: SwathsGenerator / SwathGenerator / PathPlanning`:** Direct imports failed. **Fix:** Used `dir(f2c)` to inspect the actual available classes in the installed version. Found different names like `SG_BruteForce` (for swath generation), `PP_PathPlanning` (for path planning), `HG_Const_gen` (headlands), `RP_Snake` (sorter), etc. Adjusted imports accordingly.
3.  **`NameError: 'Path' / AttributeError: _TYPE_SUPPORT`:** A name collision occurred between `nav_msgs.msg.Path` and `fields2cover.Path`. **Fix:** Imported `nav_msgs.msg.Path as RosPath` and used `RosPath` for the publisher, while using `f2c.Path` for the library's path object.
4.  **`NameError: 'contours'`:** Simple Python error. **Fix:** Ensured `self.extract_outer_boundary` was called and its result assigned to `contours` *before* the variable was used.
5.  **`TypeError: logger exc_info=True`:** The `rclpy` logger doesn't support the `exc_info` argument like standard Python logging. **Fix:** Imported the `traceback` module and used `traceback.format_exc()` to manually include traceback strings in error logs.
6.  **`TypeError: new_Field`:** Passed `f2c.Cell` to `f2c.Field` constructor. Error prototypes showed it expected `f2c.Cells` (plural collection). **Fix:** Wrapped the single `cell` in an `f2c.Cells` object: `cells_collection = f2c.Cells(cell)`. *(Initial attempt `f2c.Cells([cell])` also failed, revealing the constructor signature).*
7.  **`AttributeError: setEPSG`:** The `f2c.Field` object in this version lacked the `setEPSG` method. **Fix:** Removed the `field.setEPSG(0)` call.
8.  **`TypeError: unsupported operand type(s) for -: 'Cells' and 'Cells'`:** Tried subtracting an empty headland (`f2c.Cells`) from the field geometry (`f2c.Cells`). **Fix:** Removed the unnecessary subtraction operation `field.setField(field.getField() - no_headland)`.
9.  **`TypeError: generateBestSwaths` Argument 2 (`SGObjective`)**: Passed objective *constant* (`f2c.OBJ_NSwath`) instead of an objective *object*. **Fix:** Instantiated the objective object `sg_objective_object = f2c.OBJ_SGObjective()` and passed it. *(Initial attempt `f2c.OBJ_SGObjective(f2c.OBJ_NSwath)` failed, revealing the constructor takes no arguments).*
10. **`TypeError: generateBestSwaths` Argument 4 (`F2CCell`)**: Passed `field.getField()` (returns `f2c.Cells`) instead of the required `f2c.Cell`. **Fix:** Passed the original `cell` object.
11. **`AttributeError: planPath / generatePath`**: The `f2c.PP_PathPlanning` object didn't have these methods. **Fix:** Used `dir(path_planner)` to discover the correct method name is `searchBestPath`.
12. **`TypeError: searchBestPath` Missing Argument 'turn'**: The `searchBestPath` method required a turning strategy object. **Fix:** Imported `f2c.PP_DubinsCurves`, instantiated it (`turn_planner = f2c.PP_DubinsCurves()`), and passed it as the third argument. *(Initial attempt `f2c.PP_DubinsCurves(robot)` failed, revealing its constructor also takes no arguments).*
13. **`TypeError: searchBestPath` Argument 2 (`F2CRobot`)**: Passed `sorter` as the first argument instead of the required `robot` object. **Fix:** Corrected the argument order to `path = path_planner.searchBestPath(robot, swaths, turn_planner)`. The `sorter` object seems to be unused or used implicitly in this version's `PP_PathPlanning`.
14. **`AttributeError: getState` on `f2c.Path`**: The `f2c.Path` object does not support indexed access via `getState(i)`. **Fix:** Used `dir(path_object)` to discover the `.states` attribute. Modified `publish_path` to access `path_object.states` and iterate over that collection. *(Also confirmed via error messages that direct iteration `for state in path:` and indexing `path[i]` were not supported).*

## The Code (`coverage_planner_node.py`)
=======
# Permissions
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/tty*