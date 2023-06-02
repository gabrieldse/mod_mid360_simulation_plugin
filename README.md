# Mid360_simulation_plugin
Plugin for the simulation of the Livox Mid-360 based on the [official plugin](https://github.com/Livox-SDK/livox_laser_simulation)

Main changes:
- Support for ROS Noetic
- Support for Gazebo 11
- Standalone
    - No need to install the livox ros driver
    - No need to install the livox sdk
- Support for custom message formats
- Corrected the distortion of the point cloud

![plot](./images/undistort.png)

On the left you can see the point cloud generated by the original plugin, on the right the point cloud generated by this plugin. The distortion is clearly visible in the left image.

## Build instructions

### Tested on
| OS    | COMPILER       | Cmake version  |
| --- |----------------| -------------- |
| Ubuntu 20.04 | GCC >= 9.4     | > 3.16.3         |


1. Clone this repo into your catkin workspace src folder.
```bash
cd ~/catkin_ws/src
git clone https://github.com/fratopa/Mid360_simulation_plugin.git
```

2. Build the plugin.
```bash
cd ~/catkin_ws
source opt/ros/noetic/setup.bash
catkin build -DCMAKE_BUILD_TYPE=Release
```
## Usage instructions

To add the plugin to your robot model add the following lines to your sdf file create a link and then attach the sensor to the link as such:

```xml
<sensor type="ray" name="laser_livox">
        <pose>0 0 0.0 0 0 0</pose>
        <visualize>true</visualize>
        <always_on>True</always_on>
        <update_rate>10</update_rate>
        <!-- This ray plgin is only for visualization. -->
        <plugin name="gazebo_ros_laser_controller" filename="liblivox_laser_simulation.so">
			  <ray>
			  <scan>
				  <horizontal>
				    <samples>100</samples>
				    <resolution>1</resolution>
				    <min_angle>-3.1415926535897931</min_angle>
            <max_angle>3.1415926535897931</max_angle>
				  </horizontal>
				  <vertical>
				    <samples>50</samples>
				    <resolution>1</resolution>
				    <min_angle>-3.1415926535897931</min_angle>
            <max_angle>3.1415926535897931</max_angle>
				  </vertical>
			  </scan>
			  <range>
				  <min>0.1</min>
				  <max>40</max>
				  <resolution>1</resolution>
			  </range>
			  <noise>
				  <type>gaussian</type>
				  <mean>0.0</mean>
				  <stddev>0.0</stddev>
			  </noise>
			  </ray>
          <visualize>True</visualize>
		      <samples>20000</samples>
		      <downsample>1</downsample>
		      <csv_file_name>mid360-real-centr.csv</csv_file_name>
          <publish_pointcloud_type>2</publish_pointcloud_type>
		      <ros_topic>/livox/lidar</ros_topic>
          <frameName>base_link</frameName>
        </plugin>
      </sensor>
```
### Parameters 
The main parameters you may want to change are:
- `visualize`: If set to true the plugin will visualize the laser in gazebo, this is usefull for debugging but you may consider turning this off to improve performances.
- `downsample`: The higher the downsample factor the fewer points will be generated. this parameter can be usefull to reduce the computational load of the simulation.
- `publish_pointcloud_type`: This changes the format in which the pointcloud will be published:
    - `0`: The pointcloud will be published as a sensor_msgs::PointCloud message
    - `1`: The pointcloud will be published as a sensor_msgs::pointcloud2 message with fields `x`, `y`, `z`
    - `2` (default): The pointcloud will be published as a sensor_msgs::pointcloud2 message with fields `x`, `y`, `z`, `intensity`, `tag`, `line`. 
    - `3`: The pointcloud will be published in the Livox custom message format. 
    ```
    offset_time: 
    x: 
    y: 
    z: 
    reflectivity: 
    tag: 
    line: 

    ```

## Run instructions

To verify that the plugin is working correctly you can run the minimal example:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch livox_laser_simulation test_pattern.launch
```
This will launch the plugin with a test pattern. You should see a point cloud in RViz and a gazebo window with a spinning laser.

## Citation

If you use this plugin in your research, please cite the following paper:

```bibtex
@Article{isprs-archives-XLVIII-1-W1-2023-539-2023,
AUTHOR = {Vultaggio, F. and d'Apolito, F. and Sulzbachner, C. and Fanta-Jende, P.},
TITLE = {SIMULATION OF LOW-COST MEMS-LIDAR AND ANALYSIS OF ITS EFFECT ON THE PERFORMANCES OF STATE-OF-THE-ART SLAMS},
JOURNAL = {The International Archives of the Photogrammetry, Remote Sensing and Spatial Information Sciences},
VOLUME = {XLVIII-1/W1-2023},
YEAR = {2023},
PAGES = {539--545},
URL = {https://isprs-archives.copernicus.org/articles/XLVIII-1-W1-2023/539/2023/},
DOI = {10.5194/isprs-archives-XLVIII-1-W1-2023-539-2023}
}
```