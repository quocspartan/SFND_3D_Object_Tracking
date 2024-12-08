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
[image9]: ./media/TTC_lidar_graph.png "Lidar based TTC measurement"
[image10]: ./media/TTC_camera_graph.png "Camera based TTC measurement"
[image11]: ./media/TTC_data_variations.png "Lidar based TTC variations"
[image12]: ./media/Lidar_topview.png "Lidar top view"
[image13]: ./media/Perf_evaluation2.png "Camera based TTC evaluation"

### Match 3D Objects
Implement the method `matchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.

To match the bounding box between frames, we go through all the keypoint match pairs and associate the respective bounding boxes in both frames. We then store the unique box ID of the matches and create a bounding box match pair.

The bounding box pair with the highest number of keypoint match occurrences is then selected as the best pair.

### Compute Lidar-based TTC
The method `computeTTCLidar` is to compute the time-to-collision (TTC) in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

![alt text][image3]

The following formulars will be used for computing TTC from the lidar points

![alt text][image8]

To make the TTC more robust, I have implemented IQR method to filter out the outliers and at the end using a low pass filter to smoothen the TTC output. 

![alt text][image9]

The above chart shows the lidar based TTC over 20 consecutive frames. The TTC value after LPF is still varying a lot, however the linear line shows the time-to-collision is decreasing over time. 

### Associate Keypoint Correspondences with Bounding Boxes
The method `clusterKptMatchesWithROI` is to repare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.

All the keypoint matches belong to the current frame bounding box are grouped and filtered by eleminating the outlier matches based on Euclidean distance between them. 

<img src="media/keypointMatches/keypointMatchesOutput.gif" width="1200"  />

### Compute Camera-based TTC
The method `computeTTCCamera` is to compute the time-to-collision (TTC) for Camera. The below figure shows how to estimate the distance from camera to preceding vehicle based on the height H. 

![alt text][image2]

The following set of equations shows how to calculate the TTC using the projections from camera sensor.

![alt text][image7]

Similar to lidar based TTC, in order to make the TTC more robust, I have implemented IQR method to filter out the outliers and at the end using a low pass filter to smoothen the TTC output. 

![alt text][image10]

### Performance Evaluation 1
The lidar based TTC estimation sometimes goes wildly, as you can see the below graph.
![alt text][image11]
There maybe two reasons: 
1. Because of using the constant velocity model, so it couldn't reflect exactly when the preceding vehicle speed changes. 
2. Probably the TTC estimation based on comparing between the closest lidar points between the current frame to the previous frame. In order to improve, the averaging of distance from the lidar points may help. 
![alt text][image12]
The above graph shows the lidar points: green - curent frame, red - previous frame. 

### Performance Evaluation 2

The below graph compare the 3 best methods picked: 
1. FAST + BRIEF: time performance is good, however the TTC estimation is not stable
2. AKAZE + AKAZE: time performance is fairly ok, and the TTC estimation is quite stable. 
2. AKAZE + SIFT: time performance is fairly ok, and the TTC estimation is quite stable. 

![alt text][image13]

Image index    |  FAST_BRIEF  |    AKAZE_AKAZE   |     AKAZE_SIFT
-------------  | :----------: | :--------------: | :--------------:   
0	             |  13.518675	  |     12.55836	   |     15.967959
1	             |  13.506107   |	    13.074552    |     16.120688
2	             |  15.596969   |	    15.701079    |     16.911232
3	             |  26.963865   |	    14.281383    |     16.666767
4	             |  22.09877	  |     15.829577	   |     17.58728
5	             |  22.303338   |	    15.898827    |     17.220199
6	             |  17.623101   |	    15.972328    |     17.982874
7	             |  12.293731   |	    15.254414    |     15.962702
8	             |  15.407571   |	    14.968224    |     16.335584
9	             |  14.205937   |	    14.155646    |     17.058239
10	           |  12.605548   |    	12.086255	   |     13.510292
11	           |  15.055029   |    	13.258387	   |     14.511076
12	           |  9.027976	  |     11.881726	   |     13.246987
13	           |  10.285496   |    	11.156118	   |     12.078267
14	           |  11.306023   |    	12.27736	   |     13.619235
15	           |  10.719643   |    	13.622308	   |     12.579659
16	           |  10.899395   |    	10.637525	   |     11.132771
17	           |  8.996619	  |     10.224004	   |     10.626854
18	           |  11.627424   |    	9.18958	     |     10.598574
19	           |  9.77562     |     9.865044	   |     10.470493
