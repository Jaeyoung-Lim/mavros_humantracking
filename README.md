# mavros_humantracking

[![Build Status](https://travis-ci.com/Jaeyoung-Lim/mavros_humantracking.svg?branch=master)](https://travis-ci.com/Jaeyoung-Lim/mavros_humantracking)

`mavros_humantracking` is a package enabling image based object tracking with a gimbal attached to a drone. It uses the `MountControlPlugin` in [mavros](https://github.com/mavlink/mavros)

![multiuavsitl](mavros_humantracking/resource/humantracking.gif)

## Nodes
- humantracking_controller
    - Subscribed Topics
        - `~point_of_interest` ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))
            Point where the gimbal should be pointed at

## Simulating multiple vehicles
Multiple vehicles can be simulated with the following launcfile

```
roslaunch humantracking_controller multi_sitl_humantrack_circle.launch
```

![multiuavsitl](https://user-images.githubusercontent.com/5248102/87854473-8ec39c80-c912-11ea-946d-a3b9a062e97f.gif)
