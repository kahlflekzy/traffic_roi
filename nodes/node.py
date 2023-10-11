#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from vector_map_msgs.msg import SignalArray, VectorArray, PointArray
from math import sqrt

class Node:
    """"""
    def __init__(self):
        """"""
        self.loop_rate = 1 # check for traffic lights and draw ROIs in Hz

        self.min_distance = 0
        self.max_distance = 0
        self.roi_height = 0
        self.roi_width = 0

        self.car_pose = None
        self.signals = {}
        self.vectors = {}
        self.points = {}
        self.filtered_points = []


        self.set_parameters()

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
        rospy.sleep(2)
    
    def init_subscribers(self):
        """"""
        rospy.loginfo("Initializing subscribers.")
        rospy.Subscriber("/current_pose", PoseStamped, self.get_car_pose)
        # consider queueing so that you don't skip some
        self.signals_subscriber = rospy.Subscriber("/vector_map_info/signal", SignalArray, self.get_signals)
        self.vectors_subscriber = rospy.Subscriber("/vector_map_info/vector", VectorArray, self.get_vectors)
        self.points_subscriber = rospy.Subscriber("/vector_map_info/point", PointArray, self.get_points)
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
        rospy.spin()

    def get_signals(self, signal):
        """"""
        data = signal.data
        self.signals = {i.vid: i.type for i in data}
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
        Leave only yellow signals
        """
        yellows = [vid for vid, type_ in self.signals.items() if type_ == 2]
        pids = [self.vectors[vid] for vid in yellows]
        self.filtered_points = [self.points[pid] for pid in pids]
        rospy.loginfo("Filtered out %d points"%len(self.filtered_points))

    def find_nearest_points(self):
        """
        Using car pose, find the nearest traffic light from points\n
        Nearest is defined as 2m <= distance between car and light <= 60m
        """
        distances = [(self.euclidean(point), point) for point in self.filtered_points]
        distances = filter(lambda i: self.min_distance <= i[0] <= self.max_distance,
                           distances)
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
            # continue from here draw roi for lights
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("roi_node")
    rospy.loginfo("Initialized ROI node.")
    node = Node()
    node()