# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
  * Install Git LFS before cloning this Repo.
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Final Report

### 3D Object Tracking Overview
<img src="media/finalResult/finalOutput.gif" width="1200"  />

[//]: # (Image References)
[image1]: ./media/3D_Object_creation.png "3D Object Creation"
[image2]: ./media/Compute_TTC_from_Camera.png "Compute Camera TTC"
[image3]: ./media/Compute_TTC_from_LidarPoints.png "Compute Lidar TTC"
[image4]: ./media/Crop_and_Cluster_LidarPoints.png "Crop Cluster Lidar Points"
[image5]: ./media/Detect_And_Classify_Objects.png "Detect Classify Objects"
[image6]: ./media/KeyPoints_Scaling.png "Keypoints Scaling"
[image7]: ./media/TTC_from_Camera.png "Camera based TTC"
[image8]: ./media/TTC_from_LidarPoints.png "Lidar based TTC"

### Match 3D Objects
Implement the method `matchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.

To match the bounding box between frames, we go through all the keypoint match pairs and associate the respective bounding boxes in both frames. We then store the unique box ID of the matches and create a bounding box match pair.

The bounding box pair with the highest number of keypoint match occurrences is then selected as the best pair.

### Compute Lidar-based TTC
The method `computeTTCLidar` is to compute the time-to-collision (TTC) in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

![alt text][image3]

The following formulars will be used for computing TTC from the lidar points

![alt text][image8]

To the the TTC more robust, I have implemented IQR method to filter out the outliers and at the end using a low pass filter to smoothen the TTC output. 

### Associate Keypoint Correspondences with Bounding Boxes

### Compute Camera-based TTC

### Performance Evaluation 1

### Performance Evaluation 2