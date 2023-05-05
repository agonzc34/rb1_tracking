# rb1_tracking

## Installation

```shell
$ cd ~/ros2_ws
$ git clone --recurse-submodules https://github.com/agonzc34/rb1_tracking.git
$ rosdep install --from-paths src -r -y
$ pip3 install -r ros_yolov8/requirements.txt
$ cat ros_yolov8 | curl -O ros_yolov8/netprops/
$ colcon build
```
