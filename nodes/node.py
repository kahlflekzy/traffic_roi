#!/usr/bin/env python

import cv2
import rospy
import tf2_ros
import threading
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from utils import create_rotation_transformation_matrix, create_translation_matrix
from vector_map_msgs.msg import SignalArray, VectorArray, PointArray
from visualization_msgs.msg import Marker
from math import sqrt
import numpy as np

np.set_printoptions(precision=3)


class Node:
    """"""
    def __init__(self):
        """"""
        self.loop_rate = 5 # check for traffic lights and draw ROIs in how many circles per sec
        # populated from the ROS paramaters
        self.min_distance = 0
        self.max_distance = 0
        self.roi_height = 0
        self.roi_width = 0

        self.car_pose = None
        self.signals = {}
        self.vectors = {}
        self.points = {}
        self.filtered_points = []
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.SPHERE
        self.marker.pose.position.z = 5;
        self.marker.pose.orientation.x = 0.0;
        self.marker.pose.orientation.y = 0.0;
        self.marker.pose.orientation.z = 0.0;
        self.marker.pose.orientation.w = 1.0;
        res = 4 # used to scale up visualizations of detected traffic light points in RViz.
        self.marker.scale.x = res;
        self.marker.scale.y = res;
        self.marker.scale.z = res;

        self.marker.color.a = 1.0; 
        self.marker.color.r = 1.0;
        self.marker.color.g = 1.0;
        self.marker.color.b = 1.0;
        self.lights = 0

        self.bridge = CvBridge()

        self.superimposed_img_pub = None

        self.current_image = None
        self.camera_params = None

        self.set_parameters()

        self.transform = None
        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        
        thread = threading.Thread(target=self.tf_listener)
        thread.start()

        self.init_subscribers()
        self.init_publishers()
    
    def set_parameters(self):
        """"""
        rospy.loginfo("Getting parameters....")
        mnd = rospy.search_param('min_distance')
        self.min_distance = rospy.get_param(mnd)
        mxd = rospy.search_param('max_distance')
        self.max_distance = rospy.get_param(mxd)
        rh = rospy.search_param('roi_height')
        self.roi_height = rospy.get_param(rh)
        rw = rospy.search_param('roi_width')
        self.roi_width = rospy.get_param(rw)
        rospy.loginfo("Min Distance: %.2f"%self.min_distance)
        rospy.loginfo("Max Distance: %.2f"%self.max_distance)
        rospy.loginfo("ROI Height: %.2f"%self.roi_height)
        rospy.loginfo("ROI Width: %.2f"%self.roi_width)
        rospy.loginfo("Finished setting parameters....")

    def init_publishers(self):
        """"""
        self.superimposed_img_pub = rospy.Publisher("/camera_fl/tlr_superimpose_image", Image, queue_size=10)
        self.marker_pub = rospy.Publisher("/node/markers", Marker, queue_size=10)
        rospy.sleep(2)
    
    def init_subscribers(self):
        """"""
        rospy.loginfo("Initializing subscribers.")
        rospy.Subscriber("/current_pose", PoseStamped, self.get_car_pose)
        self.signals_subscriber = rospy.Subscriber("/vector_map_info/signal", SignalArray, self.get_signals)
        self.vectors_subscriber = rospy.Subscriber("/vector_map_info/vector", VectorArray, self.get_vectors)
        self.points_subscriber = rospy.Subscriber("/vector_map_info/point", PointArray, self.get_points)
        rospy.Subscriber("/camera_fl/decompressed/image_raw", Image, self.get_current_image)
        self.camera_subscriber = rospy.Subscriber("/camera_fl/camera_info", CameraInfo, self.get_camera_params)
        rospy.loginfo("Finished initializing subscribers.")
    
    def get_car_pose(self, data):
        """
        data has 
            .pose
                .position
                    .x
                    .y
                    .z
        """
        self.car_pose = data.pose
    
    def __call__(self):
        """"""
        rate = rospy.Rate(1)
        while len(self.signals) == 0 and len(self.vectors) == 0 and len(self.points) == 0:
            print("waiting for signals")
            rate.sleep()
        self.filter()
        self.main_loop()
        rospy.spin()

    def get_signals(self, signal):
        """"""
        data = signal.data
        self.signals = {i.vid: i for i in data}
        rospy.loginfo("Got %d signals"%len(self.signals))
        self.signals_subscriber.unregister()

    def get_vectors(self, vector):
        """"""
        data = vector.data
        self.vectors = {i.vid: i.pid for i in data}
        rospy.loginfo("Got %d vectors"%len(self.vectors))
        self.vectors_subscriber.unregister()
    
    def get_points(self, point):
        """"""
        data = point.data
        self.points = {i.pid: i for i in data}
        rospy.loginfo("Got %d points"%len(self.points))
        self.points_subscriber.unregister()

    def filter(self):
        """
        Get the points for only yellow signals
        """
        yellows = [vid for vid, signal in self.signals.items() if signal.type == 2]
        pids = [self.vectors[vid] for vid in yellows]
        self.filtered_points = [self.points[pid] for pid in pids]
        rospy.loginfo("Filtered out %d points"%len(self.filtered_points))

    def find_nearest_points(self):
        """
        Using car pose, find the nearest traffic light(s) amongst points\n
        Nearest is defined as 2m <= distance between car and light <= 60m
        """
        distances = [(self.euclidean(point), point) for point in self.filtered_points]
        distances = filter(lambda i: self.min_distance <= i[0] <= self.max_distance, distances)
        return [i[1] for i in distances]
    
    def euclidean(self, point):
        """"""
        dx = point.ly - self.car_pose.position.x
        dy = point.bx - self.car_pose.position.y
        return sqrt((dx*dx)+(dy*dy))

    def main_loop(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            lights = self.find_nearest_points()
            # rospy.loginfo("found %d lights"%len(lights))
            self.marker.header.stamp = rospy.Time.now()
            # delete previous Markers if any exists.
            self.marker.action = self.marker.DELETE
            for i in range(self.lights):
                self.marker.id = i
                self.marker_pub.publish(self.marker)
            # Add new visualization markers.
            self.marker.action = Marker.ADD;
            self.lights = len(lights)
            for index, point in enumerate(lights):
                self.marker.id = index
                output = self.point_to_list(point)
                self.marker.pose.position.x = point.ly;
                self.marker.pose.position.y = point.bx;
                coord = self.project(point=output)
                if (0 <= coord[0] <= self.camera_params.width
                    and
                    0 <= coord[1] <= self.camera_params.height):
                    # draw ROI
                    print(coord)
                self.marker_pub.publish(self.marker)
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')
                # hardcoded points, 
                # ideally this points will be generated based on coord above for each yellow signal
                start_point = (5, 5)
                end_point = (420, 420)
                color = (0, 255, 0)
                thickness = 2
                image = cv2.rectangle(cv_image, start_point, end_point, color, thickness)
                image_message = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
                self.superimposed_img_pub.publish(self.current_image)
            except:
                pass
            rate.sleep()

    def get_current_image(self, image):
        """"""
        self.current_image = image
    
    def get_camera_params(self, data):
        """"""
        self.camera_params = data

    def tf_listener(self):
        """
        Get transformations between map and camera to use for transforming traffic light points (in map frame) to the camera frame.
        """
        rospy.loginfo("Listening for map to camera_fl transforms.")
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.transform = self.tf_buffer.lookup_transform("map", "camera_fl", rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass
            rate.sleep() 
    
    def transform_point(self, point):
        """
        Transforms a point in map frame to camera frame
        """
        output = [
            point.ly - self.transform.transform.translation.x,
            point.bx - self.transform.transform.translation.y,
            point.h - self.transform.transform.translation.z,
        ]
        return output
    
    def point_to_list(self, point):
        """"""
        return [
            point.ly,
            point.bx,
            point.h,
        ]
    
    def project(self, point):
        """"""
        # create rotation transformation matrix
        xyz = self.euler_from_quaternion(self.transform.transform.rotation)
        angles = list(xyz)
        order = 'xyz'
        R = create_rotation_transformation_matrix(angles, order)
        R_ = np.identity(4)
        R_[:3, :3] = R

        # create translation transformation matrix
        offset = np.array([
            self.transform.transform.translation.x,
            self.transform.transform.translation.y,
            self.transform.transform.translation.z
        ])
        T_ = create_translation_matrix(offset)

        E = np.linalg.inv(np.matmul(R_, T_))

        # remove last row of E
        E = E[:-1, :]
        point.append(1)
        cw = np.array(point).reshape(4, 1)
        # cc is coordinates of point in camera coordinates
        cc = np.matmul(E, cw)
        # print(cc.flatten())
        # to get coordinates of point in the image
        K = np.array(self.camera_params.K).reshape(3, 3)
        p = np.matmul(K, cc).flatten()
        # print(p)
        return p

    @staticmethod
    def euler_from_quaternion(quaternion):
        """
        Converts quaternion ([x, y, z, w]) to euler roll, pitch, yaw
        import tf_convsersions
        xyz = tf_conversions.transformations.euler_from_quaternion([x, y, z, w])
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sin_r_cos_p = 2 * (w * x + y * z)
        cos_r_cos_p = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sin_r_cos_p, cos_r_cos_p)

        sin_p = 2 * (w * y - z * x)
        pitch = np.arcsin(sin_p)

        sin_y_cos_p = 2 * (w * z + x * y)
        cos_y_cos_p = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(sin_y_cos_p, cos_y_cos_p)

        return roll, pitch, yaw

if __name__ == "__main__":
    rospy.init_node("roi_node")
    rospy.loginfo("Initialized ROI node.")
    node = Node()
    node()