# mavros_humantracking

[![Build Status](https://travis-ci.com/Jaeyoung-Lim/mavros_humantracking.svg?branch=master)](https://travis-ci.com/Jaeyoung-Lim/mavros_humantracking)

`mavros_humantracking` is a package enabling image based object tracking with a gimbal attached to a drone. It uses the `MountControlPlugin` in [mavros](https://github.com/mavlink/mavros)

![multiuavsitl](mavros_humantracking/resource/humantracking.gif)

# Nodes
- humantracking_controller
    - Subscribed Topics
        - `~point_of_interest` ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))
            Point where the gimbal should be pointed at