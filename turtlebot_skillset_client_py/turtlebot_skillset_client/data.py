from .qos import HARD_STATE_QOS
import rclpy.node
import threading
from typing import Optional

from builtin_interfaces.msg import Time
from turtlebot_skillset_interfaces.msg import DataRequest

from .uid import new_id
from .qos import *

from turtlebot_skillset_interfaces.msg import DataCurrentpose, DataCurrentposeResponse


class CurrentposeData:
    def __init__(self, skillset_manager: str, subscription: bool, node: rclpy.node.Node) -> None:
        self.__event = threading.Event()
        self.__data : Optional[DataCurrentpose] = None
        self.__id : str = ""
        self.__request = node.create_publisher(DataRequest, 
            f"{skillset_manager}/turtlebot_skillset/data/currentpose/request",
            EVENT_QOS)

        if subscription:
            self.__sub = node.create_subscription(DataCurrentpose,
                f"{skillset_manager}/turtlebot_skillset/data/currentpose", 
                self.__data_callback,
                HARD_STATE_QOS)
        else:
            self.__sub = node.create_subscription(DataCurrentposeResponse,
                f"{skillset_manager}/turtlebot_skillset/data/currentpose/response", 
                self.__response_callback,
                EVENT_QOS)
        self.__subscription_flag = subscription

    def __response_callback(self, msg: DataCurrentposeResponse):
        if msg.has_data:
            data = DataCurrentpose()
            data.stamp = msg.stamp
            data.value = msg.value
            self.__data = data
        else:
            self.__data = None
        self.__event.set()

    def __data_callback(self, msg: DataCurrentpose):
            self.__data = msg

    @property
    def data(self) -> Optional[DataCurrentpose]:
        return self.__data

    def get(self) -> Optional[DataCurrentpose]:
        if self.__subscription_flag:
            return self.data
        else:
            self.__id = new_id()
            self.__event.clear()
            self.__request.publish(DataRequest(id=self.__id))
            self.__event.wait()
            return self.data


class Data:
    def __init__(self, skillset_manager: str, subscription: bool, node: rclpy.node.Node) -> None:
        
        self.__currentpose = CurrentposeData(skillset_manager, subscription, node)
        self.__data = ['currentpose']

    
    @property
    def currentpose(self) -> CurrentposeData:
        return self.__currentpose
    

    def __getitem__(self, item):
        return getattr(self, item)

    def __iter__(self):
        return iter(self.__data)

    def __len__(self):
        return len(self.__data)

    def __nonzero__(self):
        return len(self.__data) > 0
