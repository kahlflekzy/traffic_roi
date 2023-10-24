# Traffic ROI
## Introduction
Requires `numpy`,  to be installed, code might also require `tf2` installed

```
cd /root/ROS/
```

```
mkdir -p workspace/src
```

```
cd workspace/src
```

```
sudo apt-get update
```

```
sudo apt install ros-melodic-image-transport-plugins
```

```
sudo apt install ros-melodic-vector-map-msgs
```

```
sudo apt install ros-melodic-vision-opencv
```
```
sudo apt install ros-melodic-visualization-msgs
```

```
catkin_create_pkg -D "A package which detects traffic lights from a map and projects them to images from a front facing camera, based on the car's current location." -l "BSD" --author "Felix Sihitshuwam" --maintainer "Felix Sihitshuwam" traffic_roi rospy roscpp std_msgs geometry_msgs vector_map_msgs
```
Add `visualization_msgs` in find_package.
```
cd .. && catkin build traffic_roi
```

Hint to copy data from local machine to docker container, Ex.
```
docker cp config.rviz uav_lander:/root/ROS/workspace/src/traffic_roi/data/
```

## Running
Put `config.rviz`, `platform.urdf` and `traffic_lights.bag` in the `data` folder.

```
. devel/setup.bash 
```
```
roslaunch traffic_roi assignment.launch data:=data
```

## Algorithm
To solve this problem, the steps involved includes:
1. Find all traffic lights. This can be achieved by 
    > i. finding all signals, \
    > ii. finding all vectors which relates signals to points \
    > iii. finding all points and filtering points belonging to signals. This tells the location of the signals \
    > iv. Filter points to keep only points belonging to yellow signals. This makes it such that only one point on a pole is use.
2. Find the traffic lights whose points are within a euclidean distance of `min_distance` and `max_distance`. Use `x & y` coordinates. 
3. For each point (traffic light) close to the car's current location:
    > I.   Transform `point` from the `map` frame to the `camera` frame \
    > II.  Project `point` using the camera's intrinsic parameters to get the coordinates of the `point` in the image. \
    > III. Using these coordinates as centre, draw a rectangle on the image indicating ROI.
4. Loop while car is still moving or node is running.

## References
1. [Docker cp SiteCore](https://support.sitecore.com/kb?id=kb_article_view&sysparm_article=KB0383441)
2. [Docker cp Docker](https://docs.docker.com/engine/reference/commandline/cp/)
3. [Rospy Param](http://wiki.ros.org/rospy/Overview/Parameter%20Server)
4. [Adding local Repo to GitHub](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github)
5. [Visualization](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)
6. [Camera Calibration Tutorial](https://towardsdatascience.com/image-formation-and-pinhole-model-of-the-camera-53872ee4ee92)
7. [Sensor MSGS Image Example](https://www.programcreek.com/python/example/67933/sensor_msgs.msg.Image)
8. [Image Transport](http://wiki.ros.org/image_transport)
9. [Image Transport Tutorials](http://wiki.ros.org/image_transport/Tutorials)
10. [Image Common](https://wiki.ros.org/image_common)
11. [CV Bridge](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
12. [vision_cv](http://wiki.ros.org/vision_opencv)
13. [OpenCV](https://opencv.org/)
14. [OpenCV Docs](https://docs.opencv.org/3.4/)
15. [OpenCV Geometric Image Transformations](https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html)
16. [OpenCV rectangle Method](https://www.geeksforgeeks.org/python-opencv-cv2-rectangle-method/)
17. [Camera Calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
18. [Camera Info](https://www.ros.org/reps/rep-0104.html#camerainfo)
19. [ROS coordinate system](https://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions)
20. [RVIZ](http://wiki.ros.org/rviz/UserGuide)
21. [Visualizations](http://wiki.ros.org/visualization/Tutorials)
22. [Image Proc](https://wiki.ros.org/image_proc)
23. [Image Pipeline Camera Info](https://wiki.ros.org/image_pipeline/CameraInfo)

