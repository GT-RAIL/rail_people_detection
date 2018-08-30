# RAIL People Detection Metapackage

This repository contains nodes and packages that we use for person detection. All methods of person detection created by us publish people in the message formats defined in [`rail_people_detection_msgs`](rail_people_detection_msgs/). In addition, this repository contains third party people detection packages and metapackages as submodules. The nodes that we, RAIL, have written often aggregate the output from the thirdparty nodes into a more meaningful messages/representations.


## Contents

1. [`rail_people_detection_msgs`](rail_people_detection_msgs/) - the message definitions for packages within this directory.
1. [`rail_people_detector`](rail_people_detector/) - The primary package to conglomerate information about people near the robot.
1. [`rail_people_detection_utils`](rail_people_detection_utils/) - Utility classes and functions that can be useful outside the context of person detection. Eg: projecting points in RGB into the point cloud. This might become its own package/resource in the future.
1. [`leg_tracker`](leg_tracker/) - Submodule for the leg tracker showcased in [this YouTube video](https://www.youtube.com/watch?v=F2Qx3O5AyBs).
1. [`people`](people/) - Submodule for [Willow Garage's people detectors](http://wiki.ros.org/people).
