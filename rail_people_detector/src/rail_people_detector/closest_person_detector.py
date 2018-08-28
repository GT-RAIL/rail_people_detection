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
from rail_people_detection_msgs.msg import Person


class ClosestPersonDetector(object):
    """
    Merges the outputs from the leg_tracker package with the face_detector
    package from WillowGarage. Then from the combined data, it publishes the
    closest person.
    """

    def __init__(self):
        # Internal parameters
        self.listener = TransformListener()
        self.desired_pose_frame = rospy.get_param(
            "~desired_pose_frame",
            "base_link"
        )

        # Variables to keep track of state
        self.people = []
        self.people_lock = Lock()

        # The subscribers and publishers
        self.face_sub = rospy.Subscriber(
            "face_detector/people_tracker_measurements_array",
            PositionMeasurementArray,
            self.face_callback
        )
        self.leg_sub = rospy.Subscriber("people_tracked", PersonArray, self.leg_callback)
        self.person_pub = rospy.Publisher('~closest_person', Person, queue_size=10)

    def leg_callback(self, msg):
        # Acquire a lock to the people
        with self.people_lock:
            self.people = []

            # Iterate over the people and add them to those that we know
            for detected_person in msg.people:
                detected_pose = PoseStamped(header=msg.header, pose=msg.pose)
                try:
                    detected_pose = self.listener.transformPose(self.desired_pose_frame, detected_pose)
                except ExtrapolationException as e:
                    continue

                # Add the person
                person = Person(
                    header=detected_pose.header,
                    id=detected_person.id,
                    pose=detected_pose.pose
                )
                self.people.append(person)

    def face_callback(self, msg):
        # Iterate through the message and find the closest face
        closest_pos = None
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
                closest_pos = pos

        # If we have a closest face, then find the corresponding person in the
        # leg detection messages
        if closest_pos is None:
            return
        with self.people_lock:
            if len(self.people) == 0:
                return

            closest_distance = np.inf
            closest_person = None
            for person in self.people:
                distance = np.sqrt(
                    (person.pose.position.x - closest_pos.point.x) ** 2
                    + (person.pose.position.y - closest_pos.point.y) ** 2
                )
                if distance < closest_distance:
                    closest_distance = distance
                    closest_person = person

            # If we have a person, publish them!
            if closest_person is not None:
                closest_person.pose.position = closest_pos.point
                self.person_pub.publish(closest_person)

    def spin(self):
        """
        Run the detector
        """
        # All the publishing is handled by the callbacks, so we have nothing to
        # do here
        rospy.spin()
