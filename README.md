# thermal range extrinsic calibration

## gazebo simulation

you can get the simulation envrionment shown in the figure by running:

```ign gazebo 2023_aug_gazebo_world.sdf ```

![no_ray](https://github.com/allenthreee/thermal_extrinsic/blob/main/gazebo_simulation/extrinsic.png)
Click the three quotation marks in the upper right corner again, search for lidar-> visualize lidar.
Then click pause in the lower left corner to start (the triangle inside the circle) and you can see the lidar ray.
![ray](https://github.com/allenthreee/thermal_extrinsic/blob/main/gazebo_simulation/extrinsic2.png)

Then you can follow this [tutorial](https://gazebosim.org/docs/citadel/ros_integration) to convert ignition gazebo sensor info to ros_msg.
here is [my own tutorial](https://zhuanlan.zhihu.com/p/657387526)

## experiment
# thermal_range_calib
**thermal_range_calib** is a robust, high accuracy extrinsic calibration tool between thermal camera LiDAR (e.g. Livox) and RGBD-camera in targetless environment. If the scene is suitable, we can achieve high-level accuracy similar to the target based method.


## Related paper
Related paper is coming soon...


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation) and its additional ROS pacakge:

```
    sudo apt-get install ros-XXX-cv-bridge ros-xxx-pcl-conversions
```

### 1.2 **Eigen**
Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page)

### 1.3 **Ceres Solver 1.14.0**
Download Ceres 1.14.0 from [here](https://github.com/ceres-solver/ceres-solver/releases/tag/1.14.0)
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.4 **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html). (Our code is tested with PCL1.7)

## 2. Build
Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/allenthreee/thermal_range_calib.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 3. Run our example
The exmaple dataset can be download from [**Google Drive**](https://drive.google.com/drive/folders/1S8CchMg3BzNgnnZs7KR-XiLEV5juh_SD)

Move it to your local path, and then change the file path in **calib.yaml** to your data path. Then directly run
```
roslaunch thermal_range_calib calib.launch
```

## 4. Run on your own sensor set
### 4.1 Record data
Record thermal image files to png format and the point cloud to pcd files(you can record it from the simulation or hardware).
### 4.2 Modify the **calib.yaml**
Change the data path to your local data path.  
Provide the instrinsic matrix and distor coeffs for your thermal camera.


