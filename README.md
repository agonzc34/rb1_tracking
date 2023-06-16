# rb1_tracker

## Installation

```shell
$ cd ~/ros2_ws
curl https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt -o src/rb1_tracking/ros_yolov8/net_props/yolov8s.pt
curl https://raw.githubusercontent.com/ultralytics/ultralytics/main/ultralytics/tracker/cfg/bytetrack.yaml -o src/rb1_tracking/ros_yolov8/net_props/bytetrack.yaml
$ pip install -r src/rb1_tracking/python.requirements.txt
$ git clone --recurse-submodules https://github.com/agonzc34/rb1_tracking.git
$ rosdep install --from-paths src -r -y
$ pip3 install -r ros_yolov8/requirements.txt
$ cat ros_yolov8 | curl -O ros_yolov8/netprops/
$ colcon build
```

## Running
```shell
$ ros2 launch rb1_tracker_bringup rb1.launch.py
```
