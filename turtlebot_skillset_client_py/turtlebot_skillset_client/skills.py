from collections import defaultdict

from inflection import camelize
import rclpy.node
import threading
from typing import Dict
from .qos import *

from turtlebot_skillset_interfaces.msg import SkillInterrupt, SkillsetStatus
from .uid import new_id

from turtlebot_skillset_interfaces.msg import SkillGoToRequest, SkillGoToResponse, SkillGoToStatus
from turtlebot_skillset_interfaces.msg import SkillGoToInput
from turtlebot_skillset_interfaces.msg import SkillGoToProgress

class GoToSkill:
    def __init__(self, skillset_manager: str, node: rclpy.node.Node) -> None:
        self.__node = node
        self.__response_sub = node.create_subscription(SkillGoToResponse,
            f"{skillset_manager}/turtlebot_skillset/skill/go_to/response", 
            self.__response_cb,
            EVENT_QOS)
        self.__progress_sub = node.create_subscription(SkillGoToProgress,
            f"{skillset_manager}/turtlebot_skillset/skill/go_to/progress", 
            self.__progress_cb,
            SOFT_STATE_QOS)
        self.__request_pub = node.create_publisher(SkillGoToRequest,
            f"{skillset_manager}/turtlebot_skillset/skill/go_to/request",
            1)
        self.__interrupt_pub = node.create_publisher(SkillInterrupt, 
            f"{skillset_manager}/turtlebot_skillset/skill/go_to/interrupt",
            EVENT_QOS)
        self.__progress_msgs : Dict[str, SkillGoToProgress] = dict()
        self.__response_msgs : Dict[str, SkillGoToResponse] = dict()
        self.__events : Dict[str, threading.Event] = defaultdict(threading.Event)

        self.__status: SkillGoToStatus = None

    def start(self, input: SkillGoToInput) -> str:
        request = SkillGoToRequest()
        request.id = new_id()
        request.input = input
        self.__node.get_logger().debug(f"starting skill 'GoTo' with id {request.id}")
        self.__events[request.id].clear()
        self.__request_pub.publish(request)
        return request.id

    def interrupt(self, id: str):
        self.__node.get_logger().debug(f"interrupting skill 'GoTo' with id {id}")
        self.__interrupt_pub.publish(SkillInterrupt(id=id))

    def wait_result(self, id: str) -> SkillGoToResponse:
        self.__node.get_logger().debug(f"waiting skill 'GoTo' {id}")
        self.__events[id].wait()
        msg = self.__response_msgs[id]
        self.__node.get_logger().debug(f"got skill 'GoTo' response {msg}")
        return msg

    def __response_cb(self, msg: SkillGoToResponse):
        self.__node.get_logger().debug(f"received '' response {msg}")
        self.__response_msgs[msg.id] = msg
        self.__events[msg.id].set()

    def __progress_cb(self, msg: SkillGoToProgress):
        self.__node.get_logger().debug(f"received 'GoTo' progress {msg}")
        self.__progress_msgs[msg.id] = msg

    
    def update_status(self, status: SkillGoToStatus):
        self.__status = status

    def status(self) -> SkillGoToStatus:
        return self.__status

    def is_ready(self) -> bool:
        return self.__status.state == SkillGoToStatus.READY

    def is_running(self) -> bool:
        return self.__status.state == SkillGoToStatus.RUNNING

    def is_interrupting(self) -> bool:
        return self.__status.state == SkillGoToStatus.INTERRUPTING



from turtlebot_skillset_interfaces.msg import SkillGetHomeRequest, SkillGetHomeResponse, SkillGetHomeStatus
from turtlebot_skillset_interfaces.msg import SkillGetHomeInput

class GetHomeSkill:
    def __init__(self, skillset_manager: str, node: rclpy.node.Node) -> None:
        self.__node = node
        self.__response_sub = node.create_subscription(SkillGetHomeResponse,
            f"{skillset_manager}/turtlebot_skillset/skill/get_home/response", 
            self.__response_cb,
            EVENT_QOS)
        
        self.__request_pub = node.create_publisher(SkillGetHomeRequest,
            f"{skillset_manager}/turtlebot_skillset/skill/get_home/request",
            1)
        self.__interrupt_pub = node.create_publisher(SkillInterrupt, 
            f"{skillset_manager}/turtlebot_skillset/skill/get_home/interrupt",
            EVENT_QOS)
        self.__progress_msgs : Dict[str, SkillGetHomeProgress] = dict()
        self.__response_msgs : Dict[str, SkillGetHomeResponse] = dict()
        self.__events : Dict[str, threading.Event] = defaultdict(threading.Event)

        self.__status: SkillGetHomeStatus = None

    def start(self, input: SkillGetHomeInput) -> str:
        request = SkillGetHomeRequest()
        request.id = new_id()
        request.input = input
        self.__node.get_logger().debug(f"starting skill 'getHome' with id {request.id}")
        self.__events[request.id].clear()
        self.__request_pub.publish(request)
        return request.id

    def interrupt(self, id: str):
        self.__node.get_logger().debug(f"interrupting skill 'getHome' with id {id}")
        self.__interrupt_pub.publish(SkillInterrupt(id=id))

    def wait_result(self, id: str) -> SkillGetHomeResponse:
        self.__node.get_logger().debug(f"waiting skill 'getHome' {id}")
        self.__events[id].wait()
        msg = self.__response_msgs[id]
        self.__node.get_logger().debug(f"got skill 'getHome' response {msg}")
        return msg

    def __response_cb(self, msg: SkillGetHomeResponse):
        self.__node.get_logger().debug(f"received '' response {msg}")
        self.__response_msgs[msg.id] = msg
        self.__events[msg.id].set()

    
    def update_status(self, status: SkillGetHomeStatus):
        self.__status = status

    def status(self) -> SkillGetHomeStatus:
        return self.__status

    def is_ready(self) -> bool:
        return self.__status.state == SkillGetHomeStatus.READY

    def is_running(self) -> bool:
        return self.__status.state == SkillGetHomeStatus.RUNNING

    def is_interrupting(self) -> bool:
        return self.__status.state == SkillGetHomeStatus.INTERRUPTING





class Skills:
    def __init__(self, skillset_manager: str, node: rclpy.node.Node) -> None:
        
        self.__go_to = GoToSkill(skillset_manager, node)
        
        self.__get_home = GetHomeSkill(skillset_manager, node)
        self.__skills = ['go_to', 'get_home']

    
    @property
    def go_to(self) -> GoToSkill:
        return self.__go_to
    
    @property
    def get_home(self) -> GetHomeSkill:
        return self.__get_home
    

    def __getitem__(self, item):
        return getattr(self, item)

    def __iter__(self):
        return iter(self.__skills)

    def __len__(self):
        return len(self.__skills)

    def __nonzero__(self):
        return len(self.__skills) > 0
