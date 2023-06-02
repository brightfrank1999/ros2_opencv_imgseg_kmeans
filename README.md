# K-means clustering for image segmentation using OpenCV in ROS 2 Foxy

### Requirement
```
Ubuntu 20.04
ROS 2 Foxy
Python 3.8
Setuptools 45.2.0
opencv-python 4.7.0.72

A webcam
```
### Check the camera information
```

```

### Clone the package and build the node
```
cd /path/to/ros/workspace/src
git clone 

cd /path/to/workspace
colcon build --packages-select ros2foxy_opencv
source ./install/setup.bash
```

### Run the nodes 
```
ros2 run ros2_opencv

# Open a new terminal
source /opt/ros/foxy/setup.bash
source ./install/setup.bash 
ros2 run ros2_opencv_imgseg_kmeans img_subscriber
```

### Original and segmented frames
![Alt text](https://github.com/brightfrank1999/ros2_opencv_imgseg_kmeans/blob/main/img/kmeans_imgseg "Optional title")

### References
https://docs.opencv.org/3.4/d1/d5c/tutorial_py_kmeans_opencv.html

https://medium.com/towardssingularity/k-means-clustering-for-image-segmentation-using-opencv-in-python-17178ce3d6f3

https://docs.opencv.org/3.4/d1/d5c/tutorial_py_kmeans_opencv.html
