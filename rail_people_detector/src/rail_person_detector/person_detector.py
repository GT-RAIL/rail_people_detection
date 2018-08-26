#!/usr/bin/env python
# Module that receives people bounding boxes from the object detector and
# publishes the positions of people in the image

import rospy
import numpy as np

from math import isnan
from struct import unpack_from
from threading import Lock

from rail_object_detection_msgs.msg import Detections as ObjectDetections
from rail_people_detection_msgs.msg import People, Person, Centroid

from rail_person_detector.position_mapper import CentroidPositionMapper

class PersonDetector(object):
    """
    Takes the output from the object detector, and splits the output stream into
    separate topics - one for the objects and another for the people. The people
    have corresponding X,Y,Z information as well in the form of
    geometry_msgs/PointStamped
    """

    def __init__(self):
        """
        Initialize the parameters and the internal class to call the conversion
        functions
        """

        # Parameters
        self.object_detector_topic = rospy.get_param(
            "~object_detector_topic",
            "/object_detector/detections"
        )
        self.desired_position_frame = rospy.get_param(
            "~desired_position_frame",
            "base_link"
        )
        self.position_match_threshold = rospy.get_param(
            "~position_match_threshold",
            None
        )

        # Initialize the converter
        self.mapper = CentroidPositionMapper(False)

        # Output topic
        self.objects_topic = rospy.Publisher('~objects', ObjectDetections,
                                             queue_size=10)
        self.people_topic = rospy.Publisher('~people', People, queue_size=10)

        # Input topic
        self.detections_listener = rospy.Subscriber(
            self.object_detector_topic,
            ObjectDetections,
            self._handle_detections
        )

        # Helper methods and variables
        self.distance_func = lambda A,B : np.sqrt((A.x-B.x)**2 + (A.y-B.y)**2)
        self.area_func = lambda A: (
            (A.right_top_x-A.left_bot_x) * (A.left_bot_y-A.right_top_y)
        )

    def _get_valid_people(self, positions, candidates):
        """
        :param positions: list of positions of people
        :param candidates: list of bounding boxes of people
        :return: list of positions and bounding boxes that are valid
        """
        valid_people = []
        for idx,pos in enumerate(positions):
            if (np.isnan(pos.point.x)
                or np.isnan(pos.point.y)
                or np.isnan(pos.point.z)
            ):
                continue

            matched = False
            if self.desired_position_frame == 'base_link' and pos.point.z > 1.524:
                # Silva hack: Centroid is above 5 ft. Buzz is messing with us
                continue
            if self.position_match_threshold is not None:
                for i,valid in enumerate(valid_people):
                    if (self.distance_func(valid.position.point, pos.point) <= self.position_match_threshold):
                        if (self.area_func(valid.bounding_box) < self.area_func(candidates[idx].bounding_box)):
                            valid_people[i] = candidates[idx]
                            valid_people[i].position = pos
                        # Assume only one match possible
                        matched = True
                        break

            # True when no match AND when no threshold is present
            if not matched:
                candidates[idx].position = pos
                valid_people.append(candidates[idx])

        return valid_people

    def _handle_detections(self, detections):
        """
        :param detections:
        :return: nothing
        """
        objects_msg = ObjectDetections(header=detections.header)
        people_msg = People()
        candidate_people = []

        # Loop through objects and add to the people array or the objects array
        for obj in detections.objects:
            if obj.label == 'person':
                candidate_people.append(
                    Person(bounding_box=obj, bb_header=detections.header)
                )
            else:
                objects_msg.objects.append(obj)

        # Transform the people
        positions = self.mapper.convert_centroids_to_point([
            Centroid(p.bounding_box.centroid_x, p.bounding_box.centroid_y)
            for p in candidate_people
        ], self.desired_position_frame)

        # If there was some TF error, ABORT
        if positions is None:
            return

        # Validate the people in the message
        people_msg.people = self._get_valid_people(positions, candidate_people)

        # Publish the resulting topics
        self.people_topic.publish(people_msg)
        self.objects_topic.publish(objects_msg)
