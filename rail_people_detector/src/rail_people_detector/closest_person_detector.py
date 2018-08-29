#!/usr/bin/env python
# Combines people from the WG face detector to people from the leg_tracker.
# Publishes the result of this combination as rail_people_detection/People.msg

from __future__ import print_function, division

import numpy as np

from threading import Lock

import rospy
import actionlib

from tf.listener import TransformListener
from tf2_py import ExtrapolationException

from geometry_msgs.msg import PointStamped, PoseStamped
from people_msgs.msg import PositionMeasurementArray
from leg_tracker.msg import PersonArray
from rail_people_detection_msgs.msg import Person, DetectionContext


class ClosestPersonDetector(object):
    """
    Merges the outputs from the leg_tracker package with the face_detector
    package from WillowGarage. Then from the combined data, it publishes the
    closest person.
    """

    def __init__(self):
        # Internal parameters
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)
        self.listener = TransformListener()
        self.desired_pose_frame = rospy.get_param(
            "~desired_pose_frame",
            "base_link"
        )
        self.position_match_threshold = rospy.get_param("~position_match_threshold", 0.3)
        self.face_remembrance_threshold = rospy.Duration(rospy.get_param("~face_remembrance_threshold", 1.0))

        # Variables to keep track of state
        self.closest_person = None
        self.closest_person_lock = Lock()
        self.last_face_timestamp = rospy.Time(0)

        # Internal helpers
        self.person_face_distance_func = lambda A, B: np.sqrt(
            (A.pose.position.x - B.pos.x) ** 2
            + (A.pose.position.y - B.pos.y) ** 2
            + (A.pose.position.z - B.pos.z) ** 2
        )
        self.leg_detection_is_closest_face = lambda detected_person: (
            self.closest_person is not None
            and self.closest_person.detection_context.pose_source == DetectionContext.POSE_FROM_FACE
            and self.closest_person.id == str(detected_person.id)
        )

        # The subscribers and publishers
        self.face_sub = rospy.Subscriber(
            "face_detector/people_tracker_measurements_array",
            PositionMeasurementArray,
            self.face_callback
        )
        self.leg_sub = rospy.Subscriber("people_tracked", PersonArray, self.leg_callback)
        self.closest_person_pub = rospy.Publisher('~closest_person', Person, queue_size=10)

    def leg_callback(self, msg):
        closest_distance = np.inf
        closest_person = None

        # Iterate over the people and find the closest
        for detected_person in msg.people:
            detected_pose = PoseStamped(header=msg.header, pose=detected_person.pose)
            try:
                detected_pose = self.listener.transformPose(self.desired_pose_frame, detected_pose)
            except ExtrapolationException as e:
                continue

            # If this is the closest person, save them. However, if this person
            # has the same ID as the person with a face, prefer that
            person_distance = np.sqrt(detected_pose.pose.position.x ** 2 + detected_pose.pose.position.y ** 2)
            if self.leg_detection_is_closest_face(detected_person) or person_distance < closest_distance:
                closest_distance = person_distance
                closest_person = Person(
                    header=detected_pose.header,
                    id=str(detected_person.id),
                    pose=detected_pose.pose
                )

                # If the leg detection is the closest face
                if self.leg_detection_is_closest_face(detected_person):
                    break

        # Acquire a lock to the people and update the closest person's position
        # We don't want to be staring at feet...
        with self.closest_person_lock:
            if closest_person is None:
                self.closest_person = None
            elif self.closest_person is None or self.closest_person.id != closest_person.id:
                self.closest_person = closest_person
                self.closest_person.detection_context.pose_source = DetectionContext.POSE_FROM_LEGS
            else:  # self.closest_person.id == closest_person.id
                self.closest_person.pose.position.x = closest_person.pose.position.x
                self.closest_person.pose.position.y = closest_person.pose.position.y

    def face_callback(self, msg):
        # Iterate through the message and find the closest face
        closest_face = None
        closest_distance = np.inf
        for face in msg.people:
            pos = PointStamped(header=face.header, point=face.pos)
            try:
                pos = self.listener.transformPoint(self.desired_pose_frame, pos)
            except ExtrapolationException as e:
                continue

            # Find the closest face
            distance = np.sqrt(pos.point.x ** 2 + pos.point.y ** 2 + pos.point.z ** 2)
            if distance < closest_distance:
                closest_distance = distance
                closest_face = face
                closest_face.header = pos.header
                closest_face.pos = pos.point

        # Now check the distance between the closest face and the closest leg.
        # If the distance exceeds the threshold, then don't associate the face
        # with the leg
        with self.closest_person_lock:
            if closest_face is None:
                pass
            elif self.closest_person is None:
                self.last_face_timestamp = rospy.Time.now()
                self.closest_person = Person(header=closest_face.header)
                self.closest_person.pose.position = closest_face.pos
                self.closest_person.pose.orientation.w = 1.0
                self.closest_person.detection_context.pose_source = DetectionContext.POSE_FROM_FACE
            elif self.person_face_distance_func(self.closest_person, closest_face) < self.position_match_threshold:
                self.last_face_timestamp = rospy.Time.now()
                self.closest_person.pose.position = closest_face.pos
                self.closest_person.detection_context.pose_source = DetectionContext.POSE_FROM_FACE

    def spin(self):
        """
        Run the detector
        """
        # Publish the detected closest person at the desired frequency
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            # Check to see if the information on a person's face is stale
            # if rospy.Time.now() - self.last_face_timestamp > self.face_remembrance_threshold:
            #     with self.closest_person_lock:
            #         if self.closest_person is not None:
            #             self.closest_person.detection_context.pose_source = DetectionContext.POSE_FROM_LEGS

            # Otherwise, check to see if we should publish the latest detection
            with self.closest_person_lock:
                if self.closest_person is not None:
                    self.closest_person_pub.publish(self.closest_person)

            # Sleep
            rate.sleep()
