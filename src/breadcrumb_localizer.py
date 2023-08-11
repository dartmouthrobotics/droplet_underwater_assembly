#!/usr/bin/python

import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg

import rospy
import cv2
import cv2.aruco as aruco
import cv_bridge

import splinter

class BreadcrumbLocalizer(object):
    def __init__(self):
        self.eigenvalue_spline = splinter.Spline.load(self.spline_file_path)
        self.cv_bridge = cv_bridge.CvBridge()

        self.small_eigenvalue_upper_bound = 0.01
        self.marker_side_length = 0.05 # meters
        hl = self.marker_side_length / 2.0
        self.marker_local_corners = np.array([
            [-hl,  hl, 0.0],
            [ hl,  hl, 0.0],
            [ hl, -hl, 0.0],
            [-hl, -hl, 0.0],
        ])

        self.image_subscriber = None
        self.camera_info_subscriber = None
        self.position_publisher = None
        self.camera_matrix = None
        self.distortion_parameters = None
        self.marker_global_location_by_id = None

        self.aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_parameters = aruco.DetectorParameters()
        self.aruco_detector =   aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        self.subpixel_corner_refinement_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.subpixel_corner_refinement_window_size = (5,5)
        self.subpixel_corner_refinement_zero_zone = (-1,-1)

    def set_camera_info(self, message):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(message.K).reshape((3,3))
            self.distortion_parameters = np.array(message.D)

    def get_predicted_largest_eigenvalue_for_marker_position(self, marker_position):
        return self.eigenvalue_spline.eval(marker_position)

    def get_orthogonal_vector(self, vector):
        orth = np.cross(vector, np.array([1,0,0]))

        if np.linalg.norm(c1) < 0.0001:
            orth = np.cross(vector, np.array([0,1,0]))
            return orth

        return orth

    def get_predicted_covariance_for_marker_position(self, marker_position):
        largest_eigval = self.get_predicted_largest_eigenvalue_for_marker_position(marker_position)
        eigenvector_1 = self.get_orthogonal_vector(marker_position)
        eigenvector_2 = np.cross(marker_position, eigenvector_1)
        
        A = np.hstack((marker_position, eigenvector_1, eigenvector_2))
        eigenvalues = [largest_eigval, self.small_eigenvalue_upper_bound, self.small_eigenvalue_upper_bound]
        L = np.diag(eigenvalues)

        covariance_matrix = np.dot(np.dot(A, L), A.transpose())

        return covariance_matrix


    def get_marker_global_position(self, marker_id):
        return self.marker_global_location_by_id[marker_id]


    def fuse_two_marker_readings(self, position_1, position_2, cov_1, cov_2):
        # see: https://frc.ri.cmu.edu/~hpm/project.archive/reference.file/Smith&Cheeseman.pdf
        K = cov_1.dot(np.linalg.inv(cov_1 + cov_2))
        C3 = cov_1 - K.dot(cov_1)
        return position_1 + K.dot(position_2 - position_1), C3


    def fuse_marker_readings(self, marker_readings, covariances):
        if (len(marker_readings) == 0):
            rospy.logwarn("No markers detected. Cannot fuse.")
            return None, None

        if (len(marker_readings) == 1):
            rospy.logwarn("Only one marker detected")
            return marker_readings[0], covariances[0]

        fused_position, fused_cov = self.fuse_two_marker_readings(marker_readings[0], marker_readings[1], covariances[0], covariances[1])

        if (len(marker_readings) > 2):
            for i in range(2, len(marker_readings)):
                fused_position, fused_cov = self.fuse_two_marker_readings(fused_position, marker_readings[i], fused_cov, covariances[i])

        return fused_position, fused_cov


    def image_callback(self, image_message):
        if self.camera_matrix is None:
            rospy.logwarn("No camera matrix set. Cannot localize.")
            return

        # find the fiducial markers, do subpixel corner refinement, solvepnp, get predicted covariance matrix, publish the point.
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')

        corners, ids, rejectedImgPoints = self.aruco_detector.detectMarkers(cv_image)
        corners = cv2.cornerSubPix(
            cv_image, 
            corners, 
            self.subpixel_corner_refinement_window_size, 
            self.subpixel_corner_refinement_zero_zone, 
            self.subpixel_corner_refinement_criteria
        )

        marker_readings = []
        covariances = []

        for marker_id, marker_corners in zip(np.ravel(ids), corners):
            if marker_id not in self.marker_global_location_by_id:
                rospy.logwarn("Marker id {} not in marker global location by id. disregarding.".format(marker_id))
                continue

            corners_undist = cv2.fisheye.undistorPoints(
                marker_corners,
                self.camera_matrix,
                R=np.eye(3),
                P=self.camera_matrix,
            )

            _, _, tvec = cv2.solvePnP(
                self.marker_local_corners,
                corners_undist,
                self.camera_matrix,
                np.zeros((1,5)),
                cv2.SOLVEPNP_IPPE
            ) # rvec is disregarded

            covariance = self.get_predicted_covariance_for_marker_position(tvec)
            world_position = tvec + self.get_marker_global_position(marker_id)

            covariances.append(covariance)
            marker_readings.append(world_position)

    def set_fiducial_global_locations(self, message):
        pass

    def initialize_ros_objects(self):
        rospy.init_node('breadcrumb_localizer')

        self.image_subscriber = rospy.Subscriber('image', sensor_msgs.msg.Image, self.image_callback)
        self.position_publisher = rospy.Publisher('position', geometry_msgs.msg.PointStamped, queue_size=0)
        self.camera_info_subscriber = rospy.Subscriber('camera_info', sensor_msgs.msg.CameraInfo, self.set_camera_info)

    def run(self):
        self.initialize_ros_objects()
        rospy.spin()