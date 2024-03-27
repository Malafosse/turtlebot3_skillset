from .qos import HARD_STATE_QOS
import rclpy.node
import threading
from typing import Optional

from builtin_interfaces.msg import Time
from turtlebot_skillset_interfaces.msg import DataRequest

from .uid import new_id
from .qos import *




class Data:
    def __init__(self, skillset_manager: str, subscription: bool, node: rclpy.node.Node) -> None:
        self.__data = []

    

    def __getitem__(self, item):
        return getattr(self, item)

    def __iter__(self):
        return iter(self.__data)

    def __len__(self):
        return len(self.__data)

    def __nonzero__(self):
        return len(self.__data) > 0
