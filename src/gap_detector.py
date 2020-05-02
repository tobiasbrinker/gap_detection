#!/usr/bin/env python

"""
Detector for gaps in point clouds.

Notes
-----
This module implements the second part of the pipeline: The detection of gaps
using automatic thresholding, clustering and volume correction. It connects to
the first module, the preprocessing node using the 'denoised cloud' topic.
Besides several rospy.publishers to publish the point cloud in different stages
of the processing to RVIZ, this node provides the rosservice
'ugoe_gap_detection_ros/detect_gaps' to output the currently detected gaps.

"""

import visualization
import helpers

# ROS
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from ugoe_gap_detection_ros.srv import *
from ugoe_gap_detection_ros.msg import *

# Imports for dynamic reconfigure
from dynamic_reconfigure.server import Server
from ugoe_gap_detection_ros.cfg import detectorConfig

# Numpy and scikit-learn
import numpy as np
from skimage.filters import threshold_otsu, threshold_minimum, threshold_li
from skimage.filters import threshold_yen
from scipy.spatial import ConvexHull

import warnings
with warnings.catch_warnings():
    # filter sklearn\externals\joblib\parallel.py:268:
    # DeprecationWarning: check_pickle is deprecated
    warnings.simplefilter("ignore", category=DeprecationWarning)
    import sklearn.cluster as cluster
    import hdbscan


class GapDetector:
    """
    Gap detector working on the point cloud data from the preprocessing node.

    Attributes
    ----------
    preprocessed_data_sub : rospy.Subscriber
        Subscriber to the 'ugoe_gap_detection/denoised_cloud' topic of the
        preprocessing node.
    thresholded_cloud_pub : rospy.Publisher
        Publishes thresholded cloud on 'ugoe_gap_detection/thresholded_cloud'.
    clustered_cloud_pub : rospy.Publisher
        Publishes clustered cloud on 'ugoe_gap_detection/clustered_cloud'.
    convex_hull_marker_pub : rospy.Publisher
        Publishes outline marker on 'ugoe_gap_detection/convex_hull_marker'.
    centers_marker_pub : rospy.Publisher
        Publishes center point marker on 'ugoe_gap_detection/center_marker'.
    volume_text_marker_pub : rospy.Publisher
        Publishes volume text marker on 'ugoe_gap_detection/volume_text'.
    dyn_reconf_server : dynamic_reconfigure.server.Server
        Dynamic reconfigure server with config cfg/detector.cfg.
    gap_detection_srv : rospy.Service
        Gap sevice that sends current gap centers and outlines.

    Notes
    -----
    All messages sent by the publishers can be visualized using
    RVIZ(http://wiki.ros.org/rviz).

    """

    def __init__(self):
        # ROS message subscriber and publisher
        self.preprocessed_data_sub = rospy.Subscriber(
            'ugoe_gap_detection/denoised_cloud', PointCloud2,
            self.detector_callback)

        self.thresholded_cloud_pub = rospy.Publisher(
            'ugoe_gap_detection/thresholded_cloud', PointCloud2, queue_size=10)
        self.clustered_cloud_pub = rospy.Publisher(
            'ugoe_gap_detection/clustered_cloud', PointCloud2, queue_size=10)
        self.convex_hull_marker_pub = rospy.Publisher(
            'ugoe_gap_detection/convex_hull_marker', Marker, queue_size=1)
        self.centers_marker_pub = rospy.Publisher(
            'ugoe_gap_detection/center_marker', MarkerArray, queue_size=1)
        self.volume_text_marker_pub = rospy.Publisher(
            'ugoe_gap_detection/volume_text', MarkerArray, queue_size=1)

        # dynamic reconfigure server
        self.dyn_reconf_server = Server(detectorConfig, self.callback_reconf)

        # gap detection service
        self.gap_detection_srv = rospy.Service(
            'ugoe_gap_detection_ros/detect_gaps',
            GapArray, self.gap_detection_srv)

    # =============== MAIN DETECTION LOOP ===============
    def detector_callback(self, preprocessed_pc):
        """
        Main callback method of the detector that executes the procedure of
        automatic thresholding, clustering, volume correction and some
        calculations to extract the volume, boundaries and centers of the gaps.

        Parameters
        ----------
        preprocessed_pc : sensor_msgs.pointcloud2
            Preprocessed point cloud generated by the preprocessing node.

        Notes
        -----
        After each procedure step a point cloud is published over the
        publishers defined in __init__ as well as some visualization Markers
        for RVIZ.

        """
        points = helpers.PC2_to_numpy_array(preprocessed_pc)
        self.frame_id = preprocessed_pc.header.frame_id

        # ----- AUTOMATIC THRESHOLDING TO FIND GAPS -----
        depth_axis_pts = points[:, self.depth_axis]

        if(self.automatic_thresholding == 0):
            try:
                threshold = threshold_minimum(depth_axis_pts)
            except RuntimeError:
                rospy.logwarn("threshold_minimum was unable to find two " +
                              "maxima in histogram!")
                return
        elif(self.automatic_thresholding == 1):
            threshold = threshold_li(depth_axis_pts)
        elif(self.automatic_thresholding == 2):
            threshold = threshold_yen(depth_axis_pts)
        elif(self.automatic_thresholding == 3):
            threshold = threshold_otsu(depth_axis_pts, self.otsu_bins)

        device_surface_pts = points[depth_axis_pts <= threshold]
        self.surface_height = np.median(device_surface_pts[:, self.depth_axis])

        points = points[depth_axis_pts > threshold]

        # ----- PUBLISHING OF THE THRESHOLDED CLOUD -----
        thresholded_cloud = helpers.numpy_array_to_PC2(points, self.frame_id)
        self.thresholded_cloud_pub.publish(thresholded_cloud)

        # ----- CLUSTERING THE GAPS -----
        clustering_switch = {
            0: self.kmeans,
            1: self.birch,
            2: self.dbscan,
            3: self.hdbscan
        }

        cluster_algorithm = clustering_switch[self.clustering]
        labels = cluster_algorithm(points)
        labels_T = np.array([labels]).T
        clustered_points = np.append(points, labels_T, axis=1)

        color_list = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
                      [1.0, 1.0, 1.0], [0.5, 0.5, 0.5], [1.0, 0.5, 0.5],
                      [0.5, 1.0, 0.5], [0.5, 0.5, 1.0]]

        clusters = []
        colored_clusters = []
        # check for each cluster if there are enough points in the cluster
        for i in set(labels):
            cluster = clustered_points[clustered_points[:, 3] == float(i)]
            cluster = cluster[:, [0, 1, 2]]

            # create seperate list of points to visualize clusters
            for cluster_point in cluster:
                try:
                    color = np.array(color_list[i])
                except IndexError:
                    color = np.array([0.0, 0.0, 0.0])
                point_with_color = np.append(cluster_point, color)
                colored_clusters.append(point_with_color)

            # To construct a convex hull a minimum of 4 points is needed
            num_of_points, dim = cluster.shape
            if(num_of_points >= 4):
                clusters.append(cluster)

        # ----- PUBLISHING OF CLUSTERS-----
        clustered_cloud = helpers.XYZRGB_to_PC2(
            colored_clusters, self.frame_id)
        self.clustered_cloud_pub.publish(clustered_cloud)

        # ----- VOLUME CORRECTION -----
        volume_corrected_clusters = []
        num_vol_corrected_pts = 0
        volume_corrected_pts_tuple = ()
        for cluster in clusters:
            hull = ConvexHull(cluster, qhull_options="QJ")

            # Map from vertex to point in cluster
            vertices = []
            for vertex in hull.vertices:
                x, y, z = cluster[vertex]
                vertices.append([x, y, z])

            gap = cluster.tolist()
            for vertex in vertices:
                # For each vertex, add a new point to the gap with the height
                # of the surface and the other axes corresponding to the vertex
                if(self.depth_axis == 0):
                    volume_point = [self.surface_height, vertex[1], vertex[2]]
                elif(self.depth_axis == 1):
                    volume_point = [vertex[0], self.surface_height, vertex[2]]
                elif(self.depth_axis == 2):
                    volume_point = [vertex[0], vertex[1], self.surface_height]

                gap.append(volume_point)
                num_vol_corrected_pts = num_vol_corrected_pts + 1

            volume_corrected_clusters.append(np.array(gap))
            volume_corrected_pts_tuple = volume_corrected_pts_tuple + \
                (num_vol_corrected_pts,)
            num_vol_corrected_pts = 0

        # ---- CALCULATING CONVEX HULLS OF GAPS AND THEIR CENTER -----
        convex_hulls_and_info = helpers.calculate_convex_hulls_and_centers(
            volume_corrected_clusters)

        # ---- FILTER BASED ON VOLUME -----
        self.gaps = []
        for gap_info in convex_hulls_and_info:
            gap_volume = gap_info[3]

            # convert cm^3 to m^3
            gap_volume = gap_volume * 1000000.0

            if(self.min_gap_volume <= gap_volume <= self.max_gap_volume):
                self.gaps.append(gap_info)

        # ----- PUBLISHING THE MARKERS TO RVIZ -----
        if not len(self.gaps) == 0:
            gap_info = zip(*self.gaps)
            centers, vertices, simplices, volumes, num_of_points = gap_info

            convex_hull_marker = visualization.draw_convex_hull(
                simplices, self.frame_id)
            self.convex_hull_marker_pub.publish(convex_hull_marker)

            centers_marker = visualization.draw_centers(centers, self.frame_id)
            self.centers_marker_pub.publish(centers_marker)

            volume_text_marker = visualization.draw_volume_text(
                volumes, centers, self.frame_id)
            self.volume_text_marker_pub.publish(volume_text_marker)

        # ----- EVALUATION -----
        if(self.create_evaluation):
            num_of_points = np.subtract(num_of_points, num_vol_corrected_pts)
            evaluation_info = zip(num_of_points, volumes, centers)
            helpers.evaluate_detector(evaluation_info)
            self.create_evaluation = False

    # =============== DYNAMIC RECONFIGURE CALLBACK ===============
    def callback_reconf(self, config, level):
        """
        Dynamic reconfigure callback to reconfigure parameters from GUI.

        Notes
        -----
        http://wiki.ros.org/dynamic_reconfigure

        """

        self.depth_axis = config.Depthaxis
        self.min_gap_volume = config.Minimum_gap_volume
        self.max_gap_volume = config.Maximum_gap_volume

        # ----- evaluation -----
        self.create_evaluation = config.create_evaluation
        # directly uncheck checkbox to use it similar to a button
        config.create_evaluation = False

        # ----- automatic thresholding -----
        self.automatic_thresholding = config.Automatic_Thresholding
        self.otsu_bins = config.Bins

        config.groups.groups.Otsus_Method.state = False
        config.groups.groups.Yens_Method.state = False

        if(config.Automatic_Thresholding == 2):
            config.groups.groups.Yens_Method.state = True
        if(config.Automatic_Thresholding == 3):
            config.groups.groups.Otsus_Method.state = True

        # ----- clustering algorithms -----
        self.clustering = config.Clustering

        self.KM_number_of_clusters = config.number_of_clusters
        self.B_branching_factor = config.branching_factor
        self.B_threshold = config.threshold
        self.DB_eps = config.eps
        self.HDB_min_cluster_size = config.minimum_cluster_size

        config.groups.groups.K_Means.state = False
        config.groups.groups.Birch.state = False
        config.groups.groups.DBScan.state = False
        config.groups.groups.HDBSCAN.state = False

        if(config.Clustering == 0):
            config.groups.groups.K_Means.state = True
        if(config.Clustering == 1):
            config.groups.groups.Birch.state = True
        if(config.Clustering == 2):
            config.groups.groups.DBScan.state = True
        if(config.Clustering == 3):
            config.groups.groups.HDBSCAN.state = True

        rospy.loginfo("Parameters reconfigured.")
        return config

    # =============== GAP DETECTION SERVICE ===============
    def gap_detection_srv(self, req):
        """
        Callback method for the gap detection service.

        Returns
        -------
        response : GapArray
            GapArray message that contains all gaps of a device with their
            center point and outline vertices.

        Notes
        -----
        While running a the service can be called using the rosservice command:
        rosservice call /ugoe_gap_detection_ros/detect_gaps

        """

        if self.gaps is None:
            rospy.loginfo("No gaps yet, skipping")
            return []

        temporaryList = []

        for idx, gap in enumerate(self.gaps):
            center, vertices, simplices, volumes, num_of_points = gap

            gapInfo = GapInfo()

            # fill in the message
            gapInfo.gap_id = "gap" + str(idx)
            gapInfo.center_point = helpers.construct_center_pose(center)
            gapInfo.outline_poses = helpers.construct_pose_array(
                vertices, self.frame_id)

            temporaryList.append(gapInfo)

        response = GapArrayResponse()
        response.gap_array = temporaryList

        rospy.loginfo("UGOE GAP DETECTION NODE: Service response sent")
        return response

    # =============== CLUSTER ALGORITHM WRAPPERS ===============
    def kmeans(self, data):
        """
        Wrapper for sklearn.cluster.KMeans with parameters from the dynamic
        reconfigure config.

        Parameters
        ----------
        data : numpy.array
            Points in order: [[x1,y1,z1], [x2,y2,z2], ....]

        Returns
        -------
        labels : ndarray
            The cluster labels

        Notes
        -----
        https://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html

        """

        return cluster.KMeans(
            n_clusters=self.KM_number_of_clusters).fit_predict(data)

    def birch(self, data):
        """
        Wrapper for sklearn.cluster.birch with parameters from the dynamic
        reconfigure config.

        Parameters
        ----------
        data : numpy.array
            Points in order: [[x1,y1,z1], [x2,y2,z2], ....]

        Returns
        -------
        labels : ndarray
            The cluster labels

        Notes
        -----
        https://scikit-learn.org/stable/modules/generated/sklearn.cluster.Birch.html

        """

        params = {'branching_factor': self.B_branching_factor,
                  'threshold': self.B_threshold,
                  'n_clusters': None,
                  'compute_labels': True}

        return cluster.Birch(**params).fit_predict(data)

    def dbscan(self, data):
        """
        Wrapper for sklearn.cluster.dbscan with parameters from the dynamic
        reconfigure config.

        Parameters
        ----------
        data : numpy.array
            Points in order: [[x1,y1,z1], [x2,y2,z2], ....]

        Returns
        -------
        labels : ndarray
            The cluster labels

        Notes
        -----
        https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html

        """

        return cluster.DBSCAN(eps=self.DB_eps).fit_predict(data)

    def hdbscan(self, data):
        """
        Wrapper for hdbscan with parameters from the dynamic reconfigure
        config.

        Parameters
        ----------
        data : numpy.array
            Points in order: [[x1,y1,z1], [x2,y2,z2], ....]

        Returns
        -------
        labels : ndarray
            The cluster labels

        Notes
        -----
        https://hdbscan.readthedocs.io/en/latest/api.html

        """

        return hdbscan.HDBSCAN(
            min_cluster_size=self.HDB_min_cluster_size).fit_predict(data)


if __name__ == '__main__':
    rospy.init_node('gap_detector', anonymous=True)
    rospy.loginfo("Detecting Gaps...")
    GapDetector()
    rospy.spin()
