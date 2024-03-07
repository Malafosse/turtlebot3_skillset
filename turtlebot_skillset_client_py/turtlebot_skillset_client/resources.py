from importlib import resources
from turtlebot_skillset_interfaces.msg import SkillsetStatus

class Resources:
    def __init__(self) -> None:
        self.__authority : str = "Teleop"
        self.__move : str = "Idle"
        self.__home : str = "Lost"
        self.__battery_status : str = "Normal"
        self.__resources = ['authority', 'move', 'home', 'battery_status']
    
    @property
    def authority(self) -> str:
        return self.__authority
    
    @property
    def move(self) -> str:
        return self.__move
    
    @property
    def home(self) -> str:
        return self.__home
    
    @property
    def battery_status(self) -> str:
        return self.__battery_status
    

    def __getitem__(self, item):
        return getattr(self, item)

    def __iter__(self):
        return iter(self.__resources)

    def __len__(self):
        return len(self.__resources)

    def __nonzero__(self):
        return len(self.__resources) > 0
    
    def update_status(self, status: SkillsetStatus) -> None:
        for r in status.resources:
            if r.name == 'authority':
                self.__authority = r.state
            elif r.name == 'move':
                self.__move = r.state
            elif r.name == 'home':
                self.__home = r.state
            elif r.name == 'battery_status':
                self.__battery_status = r.state
            
