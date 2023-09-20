# thermal range extrinsic calibration

## gazebo simulation

you can get the simulation envrionment shown in the figure by running:

```ign gazebo 2023_aug_gazebo_world.sdf ```

![no_ray](https://github.com/allenthreee/thermal_extrinsic/blob/main/gazebo_simulation/extrinsic.png)
再点一下右上角三个引号, 搜索lidar-> visualize lidar
然后再把左下角的暂停点成启动(圆圈里面的三角形)就可一看到 lidar ray了
![ray](https://github.com/allenthreee/thermal_extrinsic/blob/main/gazebo_simulation/extrinsic2.png)

## experiment
# thermal_range_calib
**thermal_range_calib** is a robust, high accuracy extrinsic calibration tool between high resolution LiDAR (e.g. Livox) and camera in targetless environment. Our algorithm can run in both indoor and outdoor scenes, and only requires edge information in the scene. If the scene is suitable, we can achieve pixel-level accuracy similar to or even beyond the target based method.
<div align="center">
    <img src="pics/color_cloud.png" width = 100% >
    <font color=#a0a0a0 size=2>An example of a outdoor calibration scenario. We color the point cloud with the calibrated extrinsic and compare with actual image. A and C are locally enlarged
views of the point cloud. B and D are parts of the camera image
corresponding to point cloud in A and C.</font>
</div>

## Info
New features:
1. Support muti-scenes calibration (more accurate and robust)

## Related paper
Related paper is coming soon


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
git clone @todo
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
You will get the following result. (Sensor suite: Livox Avia + Realsense-D435i)
<div align="center">
    <img src="pics/single_calib_case.png" width = 100% >
    <font color=#a0a0a0 size=2>An example of single scene calibration.</font>
</div>

## 4. Run on your own sensor set
### 4.1 Record data
Record the point cloud to pcd files and record image files.
### 4.2 Modify the **calib.yaml**
Change the data path to your local data path.  
Provide the instrinsic matrix and distor coeffs for your camera.

### 4.3 Use multi scenes calibration
Change the params in **multi_calib.yaml**, name the image file and pcd file from 0 to (data_num-1).

