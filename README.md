<div align="center" id="top"> 
  <img height="70px" src="https://blog.generationrobots.com/wp-content/uploads/2016/03/Logo-ROS-Robot-Operating-System1.jpg" alt="ROS" />
  <img height="70px" style="background-color:white" src="https://pointclouds.org/assets/images/logo.png" alt="PointCloud" />
  &#xa0;

  <!-- <a href="https://ros.netlify.app">Demo</a> -->
</div>

<h1 align="center">3D Object Detection and Recognition Based on RGBD Images</h1>

<p align="center">
  <img alt="Github top language" src="https://img.shields.io/github/languages/top/malikmal/object_recognition_pkg?color=56BEB8">

  <img alt="Github language count" src="https://img.shields.io/github/languages/count/malikmal/object_recognition_pkg?color=56BEB8">

  <img alt="Repository size" src="https://img.shields.io/github/repo-size/malikmal/object_recognition_pkg?color=56BEB8">

  <img alt="License" src="https://img.shields.io/github/license/malikmal/object_recognition_pkg?color=56BEB8">

  <img alt="Github issues" src="https://img.shields.io/github/issues/malikmal/object_recognition_pkg?color=56BEB8" />

  <img alt="Github forks" src="https://img.shields.io/github/forks/malikmal/object_recognition_pkg?color=56BEB8" />

  <img alt="Github stars" src="https://img.shields.io/github/stars/malikmal/object_recognition_pkg?color=56BEB8" />
</p>

<!-- Status -->

<!-- <h4 align="center"> 
	🚧  Paper still reviewed...  🚧
</h4>  -->

<hr>

<p align="center">
  <a href="#dart-about">About</a> &#xa0; | &#xa0; 
  <a href="#sparkles-features">Features</a> &#xa0; | &#xa0;
  <a href="#rocket-technologies">Technologies</a> &#xa0; | &#xa0;
  <a href="#white_check_mark-requirements">Requirements</a> &#xa0; | &#xa0;
  <a href="#checkered_flag-starting">Starting</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
  <a href="https://github.com/malikmal" target="_blank">Author</a>
</p>

<br>

## :dart: About ##

Package for 3D Object Detection and Recognition Based on RGBD Images in Robot Operating System (ROS) Workspace. This is my final project as student in EEPIS/PENS.


## :link: Paper ##

https://ieeexplore.ieee.org/document/9594034
```
@INPROCEEDINGS{9594034,
  author={Birri, Ikmalil and Dewantara, Bima Sena Bayu and Pramadihanto, Dadet},
  booktitle={2021 International Electronics Symposium (IES)}, 
  title={3D Object Detection and Recognition based on RGBD Images for Healthcare Robot}, 
  year={2021},
  volume={},
  number={},
  pages={173-178},
  doi={10.1109/IES53407.2021.9594034}}
```

## 📹: Publication ##
[![Watch the video](https://img.youtube.com/vi/T49SgeLJDdc/maxresdefault.jpg)](https://www.youtube.com/watch?v=T49SgeLJDdc)

## :sparkles: Features ##

:heavy_check_mark: Feature 1; Realtime RGBD Camera using Intel LibRealsense and opencv\
:heavy_check_mark: Feature 2; Filter and Segmentation using Remove NaN, passthrough, VoxelGrid downsample, and RANSAC Plane Segmentation\
:heavy_check_mark: Feature 3; Detection or Clustering using Euclidean Cluster\
:heavy_check_mark: Feature 4; Prediction Artificial Neural Network using FANN Library\
:heavy_check_mark: Feature 5; Visualize 3D Bounding Box and label using Visualizer PCL\

## :rocket: Technologies ##

The following tools were used in this project:

- [PointCloud](https://pcl.readthedocs.io/)
- [FANN](http://leenissen.dk/fann/wp/)
- [LibRealsense](https://github.com/IntelRealSense/librealsense/)
- [OpenCV](https://opencv.org/)
- [ROS](https://ros.org)


## :white_check_mark: Requirements ##

Before starting :checkered_flag:, you need to have [Git](https://git-scm.com), [ROS](https://ros.org) and [PCL](https://pcl.readthedocs.io) installed.

## :checkered_flag: Starting ##

```bash
# make sure inside catkin_ws/src
$ cd catkin_ws/src

# Clone this project
$ git clone https://github.com/Malikmal/object_recognition_pkg

# Access
$ cd ../

# Build the project (workspace)
$ catkin_make

# Prepare datasets
# contains folders of object scenes

# Run training
$ rosrun object_recognition_pkg main_trainning folder_name

# Run Testing offline file mode
$ rosrun object_recognition_pkg main src/data/scene_mug_table.pcd

# Run realtime camera 
$ rostopic
$ rosrun object_recognition_pkg camera_node
$ rosrun object_recognition_pkg main_camera
```


## :memo: License ##

<!-- This project is under license from MIT. For more details, see the [LICENSE](LICENSE.md) file. -->


Made with :heart: by <a href="https://github.com/{{YOUR_GITHUB_USERNAME}}" target="_blank">[Ikmalil Birri](https://www.linkedin.com/in/ikmalil-birri-99b11611a/)</a>


&#xa0;

<a href="#top">Back to top</a>
