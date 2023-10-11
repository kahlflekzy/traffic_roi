#!/usr/bin/env python

from typing import Any
import rospy
from geometry_msgs.msg import PoseStamped
from vector_map_msgs.msg import SignalArray, VectorArray, PointArray

class Node:
    """"""
    def __init__(self) -> None:
        """"""
        self.min_distance = 0
        self.max_distance = 0
        self.roi_height = 0
        self.roi_width = 0

        self.car_pose = None
        self.signals = {}
        self.vectors = {}
        self.points = {}


        self.set_parameters()

        self.init_subscribers()
        self.init_publishers()
    
    def set_parameters(self):
        """"""
        rospy.loginfo("Getting parameters.")
        mnd = rospy.search_param('min_distance')
        self.min_distance = rospy.get_param(mnd)
        mxd = rospy.search_param('max_distance')
        self.max_distance = rospy.get_param(mxd)
        rh = rospy.search_param('rh')
        self.roi_height = rospy.get_param(rh)
        rw = rospy.search_param('rw')
        self.roi_width = rospy.get_param(rw)
        rospy.loginfo("Finished setting parameters.")

    def init_publishers(self):
        """"""
        rospy.sleep(2)
    
    def init_subscribers(self):
        """"""
        rospy.loginfo("Initializing subscribers.")
        rospy.Subscriber("/current_pose", PoseStamped, self.get_car_pose)
        # consider queueing so that you don't skip some
        rospy.Subscriber("/vector_map_info/signal", SignalArray, self.get_signals)
        rospy.Subscriber("/vector_map_info/vector", VectorArray, self.get_vectors)
        rospy.Subscriber("/vector_map_info/point", PointArray, self.get_points)
        rospy.loginfo("Finished initializing subscribers.")
    
    def get_car_pose(self, data):
        """"""
        self.car_pose = data
    
    def __call__(self) -> Any:
        """"""
        rospy.spin()

    def get_signals(self, data: SignalArray):
        """"""
        self.signals[data.vid] = data.type

    def get_vectors(self, data: VectorArray):
        """"""
        self.vectors[data.vid] = data.pid
    
    def get_points(self, data: PointArray):
        """"""
        self.points[data.pid] = data



if __name__ == "__main__":
    rospy.init_node("roi_node")
    rospy.loginfo("Initialized ROI node.")
    node = Node()
    node()