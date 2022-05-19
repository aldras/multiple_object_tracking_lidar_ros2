# Multiple objects detection, tracking and classification from LIDAR scans/point-clouds


[![DOI](https://zenodo.org/badge/47581608.svg)](https://zenodo.org/badge/latestdoi/47581608)

![Sample demo of multiple object tracking using LIDAR scans](https://media.giphy.com/media/3YKG95w9gu263yQwDa/giphy.gif)

PCL based ROS package to Detect/Cluster --> Track --> Classify static and dynamic objects in real-time from LIDAR scans implemented in C++.

### Features:

- K-D tree based point cloud processing for object feature detection from point clouds
- Unsupervised euclidean cluster extraction (3D) or k-means clustering based on detected features and refinement using RANSAC (2D)
- Stable tracking (object ID & data association) with an ensemble of Kalman Filters 
- Robust compared to k-means clustering with mean-flow tracking
- Ported to ROS 2 Foxy Fitzroy on 2022-05-20.
- Original is located at `https://github.com/praveen-palanisamy/multiple-object-tracking-lidar.git`
- Commit 27db548 on Aug 30, 2021 was used for the port.
### Usage:

Follow the steps below to use this (`multi_object_tracking_lidar`) package:

1. Create a workspace. `mkdir ~/ws_ros2/src`
1. Navigate to the `src` folder in your catkin workspace: `cd ~/ws_ros2/src`
1. Clone this repository: `git clone https://github.com/aldras/multiple-object-tracking-lidar.git`
1. Compile and build the package: `cd ~/ws_ros2 && colcon build`
1. Add the catkin workspace to your ROS environment: `source ~/ws_ros2/install/setup.bash`
1. Run the `kf_tracker` ROS node in this package: `ros2 launch multiple_object_tracking_lidar multiple_object_tracking_lidar.launch.py`

Change parameters in the launch file launch/multiple_object_tracking_lidar.launch.py for the frame_id and filtered_cloud.

If all went well, the ROS node should be up and running! As long as you have the point clouds published on to the `filtered_cloud` rostopic, you should see outputs from this node published onto the `obj_id`, `cluster_0`, `cluster_1`, …, `cluster_5` topics along with the markers on `viz` topic which you can visualize using RViz.

### Supported point-cloud streams/sources:
The input point-clouds can be from:
1. A real LiDAR or 
2. A simulated LiDAR or 
3. A point cloud dataset or 
4. Any other data source that produces point clouds

**Note:** This package expects valid point cloud data as input. The point clouds you publish to the "`filtered_cloud`" is **not** expected to contain NaNs. The point cloud filtering is somewhat task and application dependent and therefore it is not done by this module. 
PCL library provides `pcl::removeNaNFromPointCloud (...)` method  to filter out NaN points. You can refer to [this example code snippet](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/issues/29#issuecomment-672098760) to easily filter out NaN points in your point cloud.

**Note:**
After porting to ROS 2 there are issues with removal of support for Marker and MarkerArray, therefore it is currently commented out.
Lines 405 and 547 "ec.setClusterTolerance(0.3)" had been changed to "ec.setClusterTolerance(0.08)", to resolve speed issues.

## Citing

If you use the code or snippets from this repository in your work, please cite:

```bibtex
@software{praveen_palanisamy_2019_3559187,
  author       = {Praveen Palanisamy},
  title        = {{praveen-palanisamy/multiple-object-tracking-lidar: 
                   Multiple-Object-Tracking-from-Point-Clouds_v1.0.2}},
  month        = dec,
  year         = 2019,
  publisher    = {Zenodo},
  version      = {1.0.2},
  doi          = {10.5281/zenodo.3559187},
  url          = {https://doi.org/10.5281/zenodo.3559186}
}
```

### Wiki

[Checkout the Wiki pages](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/wiki)

1. [Multiple-object tracking from pointclouds using a Velodyne VLP-16](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/wiki/velodyne_vlp16)
