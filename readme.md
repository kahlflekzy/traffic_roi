# Traffic ROI
## Introduction
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
catkin_create_pkg -D "A package which detects traffic lights from a map and projects them to images from a front facing camera, based on the car's current location." -l "BSD" --author "Felix Sihitshuwam" --maintainer "Felix Sihitshuwam" traffic_roi rospy roscpp std_msgs geometry_msgs vector_map_msgs
```
```
cd .. && catkin build traffic_roi
```

Copy Data from local to docker image, Ex.
```
docker cp config.rviz uav_lander:/root/ROS/workspace/src/traffic_roi/data/
```

## Running

```
. devel/setup.bash 
```
```
roslaunch traffic_roi assignment.launch path:=data
```
## References
1. [Docker cp SiteCore](https://support.sitecore.com/kb?id=kb_article_view&sysparm_article=KB0383441)
2. [Docker cp Docker](https://docs.docker.com/engine/reference/commandline/cp/)
3. [Rospy Param](http://wiki.ros.org/rospy/Overview/Parameter%20Server)
4. [Adding local Repo to GitHub](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github)