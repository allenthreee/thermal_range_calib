# thermal range extrinsic calibration

## gazebo simulation
You can find the simulation environment in this [repo](https://github.com/allenthreee/thermal_range_calib_simulation) 

## experiment
# thermal_range_calib
**thermal_range_calib** is a robust, high-accuracy extrinsic calibration tool between thermal camera LiDAR (e.g. Livox) and RGBD-camera in targetless environment. If the scene is suitable, we can achieve high-level accuracy similar to the target-based method.

The system architecture is shown in the following diagram:

![6971701232187_ pic](https://github.com/allenthreee/thermal_range_calib/assets/59171742/9f0a9377-5a5e-40fb-ac9f-1d327cab06e3)

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

our thermal camera intrinsic is shown below:


![thermal_camera_intrinsics_fx_fy_00](https://github.com/allenthreee/thermal_range_calib/assets/59171742/44b04117-4de3-4bc2-94b6-a5fdef8bedb6)


![thermal_camera_intrinsics_cx_cy_00](https://github.com/allenthreee/thermal_range_calib/assets/59171742/a524c0e2-d9aa-4f69-a9fd-3f9bf6035d79)

example data could be download here: [google drive](https://drive.google.com/drive/folders/1qVTr8TDtHo3dQR90_XpZD-76ECm691wW?usp=sharing)

Move it to your local path, and then change the file path in **thermal_calib.yaml** to your data path. Then directly run
```
roslaunch thermal_range_calib thermal_calib.launch
```

Following are some fusion examples:

![6951701231997_ pic](https://github.com/allenthreee/thermal_range_calib/assets/59171742/354e0e90-bd94-49ba-aa44-12f50a86da49)

![6961701231999_ pic](https://github.com/allenthreee/thermal_range_calib/assets/59171742/50cf6868-36bf-49b5-8156-fdf3ee010f9a)


## 4. Run on your own sensor set
### 4.1 Record data
Record thermal image files to png format and the point cloud to pcd files(you can record it from the simulation or hardware).
### 4.2 Modify the **calib.yaml**
Change the data path to your local data path.  
Provide the instrinsic matrix and distor coeffs for your thermal camera. 

## 5. Acknowledgements
Thanks for [Livox_Camera_calib](https://github.com/hku-mars/livox_camera_calib)(C. Yuan et al. Pixel-level extrinsic self calibration of high resolution lidar and camera in targetless environments).


