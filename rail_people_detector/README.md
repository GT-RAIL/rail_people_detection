# RAIL People Detector

Package to integrate detections from multiple sensory modalities and packages into a single stream of information about people near the robot.


## Two Minute Intro

This package has two nodes:

1. `people_from_objects_node`: Given an object detections topic, this node looks for objects of type **person** and filters them into a separate topic and the remaining objects into another topic.
1. `closest_person_node`: Given detections from the [ICRA leg detector](https://github.com/petschekr/leg_tracker/tree/fetch) and [Willow Garage's face detector](http://wiki.ros.org/face_detector), this node merges them together and outputs the nearest person to the robot.

##### People From Objects

Assuming that object detections are on the topic `/object_detector/detections` and are of the type `rail_object_detection_msgs/Detections.msg`:

```
rosrun rail_people_detector people_from_objects_node
```

People will be output on the topic `people_from_objects_node/people` with the type [`rail_people_detection_msgs/People.msg`](../rail_people_detection_msgs/msg/People.msg). The other objects will be output on the topic `people_from_objects_node/objects` with the type `rail_object_detection_msgs/Detections.msg`.

#### Closest Person Node

```
roslaunch rail_people_detector closest_person_detection.launch start_all:=true
```

This command starts up the leg tracker, the face detector, and the `closest_person_node`. The closest person to the robot, if available, will be published on the topic `rail_people_detector/closest_person` with the type [`rail_people_detection_msgs/Person.msg`](../rail_people_detection_msgs/msg/Person.msg).

Start the launch file with `start_all:=false` (default behaviour) if you already have the leg detector and face detector running.


## Menu
 * [Installation](#installation)
 * [ROS Nodes](#ros-nodes)
 * [ROS Launch Files](#ros-launch-files)
 * [Scope for Improvement](#scope-for-improvement)


## Installation

1. Put this repository into your workspace
1. Initialize the submodules: `git submodule init && git submodule update`.
1. Run `catkin_make` and enjoy (slightly more convenient) people detection!


## ROS Nodes

Here, we go into more detail on the nodes in this package.

### people_from_objects_node

Relevant topics and parameters are as follows:

* **Parameters**
  * `object_detector_topic` (`string`, default: "/object_detector/detections")
  <br/>Object detector topic with incoming object detections
  * `desired_pose_frame` (`string`, default: "base_link")
  <br/>The tf frame that all the detected people's poses should be reported in.
  * `position_match_threshold` (`float`, default: `None`)
  <br/>If multiple detections of the same person are likely, this determines the (3D) distance between those two detections below which they will be considered the same person. `None` signals that this test should be disabled.
  * `point_cloud_topic` (`string`, default: `/kinect/qhd/points`)
  <br/>The point cloud to use when projecting the person in the RGB image into the 3D world.
  * `point_cloud_frame` (`string`, default: `kinect_rgb_optical_frame`)
  <br/>The expected tf frame of the point cloud.
* **Topics**
  * `<param.object_detector_topic>` (`rail_object_detection_msgs/Detections`)
  <br/>Incoming topic with object detections.
  * `<node_name>/objects` (`rail_object_detection_msgs/Detections`)
  <br/>Filtered topic of object detections without any people in it.
  * `<node_name>/people` ([`rail_people_detection_msgs/People`](../rail_people_detection_msgs/msg/People.msg))
  <br/>The people detected in the object detections, and projected into the 3D world using the point cloud

### closest_person_node

Relevant topics and parameters are as follows:

* **Parameters**
  * `publish_rate` (`float`, default: 10.0)
  <br/>The rate at which the closest person to the robot is published
  * `desired_pose_frame` (`string`, default: "base_link")
  <br/>The tf frame that the detected person's poses should be reported in.
  * `position_match_threshold` (`float`, default: 1.0)
  <br/>The maximum 3D distance below which a face detection is considered to be associated to a leg detection. Leg detections usually have a `Z: 0` (and people are generally >= 1.5m tall), so there is a hack in place that hardcodes a `Z: 1.2 (4ft)` to all leg detections for the purposes of the match calculation. Otherwise, faces have the potential to be associated with incorrect leg detections.
  * `debug` (`bool`, default: `False`)
  <br/>Flag to determine if debug visualization markers should be published.
* **Topics**
  * `face_detector/people_tracker_measurements_array` (`people_msgs/PositionMeasurementArray`)
  <br/>Expected incoming topic with face detections.
  * `people_tracked` (`leg_tracker/PersonArray`)
  <br/>Expected incoming topic with leg detections.
  * `<node_name>/closest_person` ([`rail_people_detection_msgs/Person`](../rail_people_detection_msgs/msg/Person.msg))
  <br/>The closest person detected from fusing data in leg and face detections. By default, all leg detections are candidates to be the closest person. The closest face detection is only a candidate if it is also associated with a leg detection.
  * `<node_name>/debug/1` (`visualization_msgs/Marker`)
  <br/>The closest face detection, represented as a green sphere.
  * `<node_name>/debug/2` (`visualization_msgs/Marker`)
  <br/>The closest leg detection, represented as a red sphere.


## ROS Launch Files

This section details the different launch files in this package. While some launch the nodes created by `rail_people_detector`, others are wrapper launch files to help launch third party detectors.

### closest_person_detection.launch

Launches the `closest_person_node` (named `rail_people_detector`), and optionally, also launches the `leg_tracker` and the `face_detector`. Its args are:

* `start_all`. Default: `false`. Boolean to launch the accompanying leg and face detectors.
* `face_detector`. Default: `$(arg start_all)`. Boolean to just launch the face detector.
* `leg_detector`. Default: `$(arg start_all)`. Boolean to just launch the leg detector.
* `robot_frame`. Default: `base_link`. All distance checks will be made relative to this frame. People will also be published relative to this frame.
* `debug`. Default: `false`. Publish debug topics?
* All the params in [`wg_face_detector.launch`](#wg_face_detectorlaunch)

### icra_leg_tracker.launch

Launches the leg detector and tracker. It takes no args.

### wg_face_detector.launch

Launches Willow Garage's face detector. Its args are:

* `camera`. Default: `head_camera`. Combined together, these topics specify the topics for the RGB and depth camera streams. With the default values, the topics resolve to `/head_camera/rgb/image_rect_color` and `/head_camera/depth_registered/image_raw`.
* `rgb_ns`. Default: `rgb`. Combined together, these topics specify the topics for the RGB and depth camera streams. With the default values, the topics resolve to `/head_camera/rgb/image_rect_color` and `/head_camera/depth_registered/image_raw`.
* `depth_ns`. Default: `depth_registered`. Combined together, these topics specify the topics for the RGB and depth camera streams. With the default values, the topics resolve to `/head_camera/rgb/image_rect_color` and `/head_camera/depth_registered/image_raw`.
* `image_topic`. Default: `image_rect_color`. Combined together, these topics specify the topics for the RGB and depth camera streams. With the default values, the topics resolve to `/head_camera/rgb/image_rect_color` and `/head_camera/depth_registered/image_raw`.
* `depth_topic`. Default: `image_raw`. Combined together, these topics specify the topics for the RGB and depth camera streams. With the default values, the topics resolve to `/head_camera/rgb/image_rect_color` and `/head_camera/depth_registered/image_raw`.
* `camera_frame`. Default: `head_camera_rgb_optical_frame`. The tf frame of the camera images.
* `paramfile_classifier`. Default: `$(find face_detector)/param/classifier.yaml`. The configuration file for the face detector.
* `debug`. Default: `false`. Boolean to indicate whether to publish debug topics.


## Scope for Improvement

- [ ] Integrate pose estimates from the [`rail_pose_estimation`](https://github.com/GT-RAIL/rail_pose_estimation/) into the information about people.
- [ ] Add the ability to integrate information from all three sources of person detections - Object Detectors, Face Detectors, Leg Detectors.
- [ ] Update face detector code to be agnostic of the face detector that is actually in use - Willow Garage or RAIL.
- [ ] Improve the hacks in the data association in `closest_person_node`.
