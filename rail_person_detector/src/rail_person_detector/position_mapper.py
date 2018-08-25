#!/usr/bin/env python
# Module that takes a maps a point in an image to the desired reference frame

import rospy
import tf
import numpy as np

from math import isnan
from struct import unpack_from
from threading import Lock

from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2
from rail_people_detection_msgs.srv import CentroidPositionQuery, \
    CentroidPositionQueryResponse

class CentroidPositionMapper(object):
    """
    This class takes the centroid of an object as X,Y pixel values and
    returns the position of that object in the desired reference frame.
    Requires that the point cloud be present
    """

    def __init__(self, use_service=True):
        """
        Initialize parameters and listeners
        :param use_service: Boolean for whether a separate service should be
            initialized to communicate with this node
        """
        self.use_service = use_service

        # Parameters
        self.pcl_topic = rospy.get_param("~kinect_pcl_topic",
                                         "/kinect/qhd/points")
        self.pcl_frame = rospy.get_param("~kinect_pcl_frame",
                                         "kinect_rgb_optical_frame")
        self.transform_wait_timeout = rospy.get_param(
            "~transform_wait_timeout", 2.0)

        # Storage
        self.point_cloud = None # Stored point cloud
        self.pc_point_size = None
        self.pc_row_size = None
        self.pc_width = None
        self.pc_height = None
        self.point_cloud_lock = Lock()

        # Listeners
        self.tf_listener = tf.TransformListener()
        self.pcl_listener = rospy.Subscriber(self.pcl_topic, PointCloud2,
                                             self._save_point_cloud)
        if self.use_service:
            self.query_server = rospy.Service("~query", CentroidPositionQuery,
                                              self._handle_centroid_query)

    def _save_point_cloud(self, point_cloud):
        """
        Stores the point cloud for future processing as needed
        :param point_cloud:
        :return: nothing
        """
        if self.point_cloud_lock.acquire(False):
            self.point_cloud = point_cloud
            self.pc_point_size = point_cloud.point_step
            self.pc_row_size = point_cloud.row_step
            self.pc_width = point_cloud.width
            self.pc_height = point_cloud.height
            self.point_cloud_lock.release()

    def convert_centroids_to_point(self, centroids, desired_frame):
        """
        Handles requests for centroid locations
        :param centroids: List of centroids that we need a position for
        :param desired_frame: Desired TF frame for the output positions
        :return: List of positions of the corresponding centroids. None if fail
        """

        # Calculate the position of the corresponding point in the point cloud
        pcl_points = []
        self.point_cloud_lock.acquire()
        if self.point_cloud is None:
            self.point_cloud_lock.release()
            return []
        header = self.point_cloud.header # Use this header later
        for centroid in centroids:

            # Check that the point is not at the boundaries
            if (centroid.x >= self.pc_width-1
                or centroid.x < 0
                or centroid.y >= self.pc_height-1
                or centroid.y < 0
            ):
                pcl_points.append(None)
                continue

            # The position of the point is actually the mean of nearby points
            neighbours = [(centroid.x, centroid.y),
                          (centroid.x, centroid.y+1),
                          (centroid.x+1, centroid.y),
                          (centroid.x+1, centroid.y+1)]

            # Only use those points for which we do have positions
            num_valid = 0
            neighbour_pos = np.empty((len(neighbours), 3))
            for neighbour in neighbours:
                x_idx = (self.pc_row_size * neighbour[1]
                         + neighbour[0] * self.pc_point_size)
                y_idx = (self.pc_row_size * neighbour[1]
                         + neighbour[0] * self.pc_point_size + 4)
                z_idx = (self.pc_row_size * neighbour[1]
                         + neighbour[0] * self.pc_point_size + 8)
                float_x = unpack_from('f', self.point_cloud.data, x_idx)
                float_y = unpack_from('f', self.point_cloud.data, y_idx)
                float_z = unpack_from('f', self.point_cloud.data, z_idx)

                if isnan(float_x[0]) or isnan(float_y[0]) or isnan(float_z[0]):
                    continue

                neighbour_pos[num_valid, :] =\
                    (float_x[0], float_y[0], float_z[0])
                num_valid += 1

            # Don't consider this point if none of the nearby points have a
            # position in the point cloud
            if num_valid == 0:
                pcl_points.append(None)
            else:
                pcl_points.append(
                    np.mean(neighbour_pos[0:num_valid,:], axis=0)
                )

        self.point_cloud_lock.release()

        # Calculate the transformed position for all the valid points. For
        # the rest, return a None object that has a timestamp of 0 and values
        # of NAN
        response = []
        for pcl_point in pcl_points:

            # Handle the case when the point is invalid
            if pcl_point is None:
                zp = PointStamped()
                zp.point = Point(*([float('nan')] * 3))
                response.append(zp)
            else: # Transform the point to the desired reference frame
                try:
                    self.tf_listener.waitForTransform(
                        self.pcl_frame,
                        desired_frame,
                        rospy.Time(0),
                        rospy.Duration(self.transform_wait_timeout)
                    )
                    (trans, rot) = self.tf_listener.lookupTransform(
                        self.pcl_frame,
                        desired_frame,
                        rospy.Time(0)
                    )
                except tf.Exception as e:
                    rospy.logerr("Error transforming %s to %s: %s"
                                 % (self.pcl_frame, desired_frame, str(e)))
                    return None

                pcl_R_desired =\
                    tf.transformations.quaternion_matrix(rot)[0:3,0:3]
                pcl_p_desired = (np.dot(pcl_point, pcl_R_desired)
                                 - np.dot(trans, pcl_R_desired))

                tp = PointStamped(header=header, point=Point(*pcl_p_desired))
                tp.header.frame_id = desired_frame
                response.append(tp)

        # Return the response
        return response

    def _handle_centroid_query(self, req):
        """
        Handles requests for centroid locations
        :param req: List of centroids that we need a position for
        :return: A CentroidPositionQueryResponse object or None if failed
        """
        response = self.convert_centroids_to_point(req.centroids,
                                                   req.desired_frame)
        # Return the response object
        return CentroidPositionQueryResponse(response)
