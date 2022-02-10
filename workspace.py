import dros_utils
from typing import Optional

class Workspace:
    def __init__(self: Workspace, name: str, ros_version="melodic") -> Workspace:
        self.name = name
        self.ros_version = ros_version


    def init(self: Workspace, path: Optional[str]=None) -> None:
        dros_utils.new(self.workspace, self.ros_version, path)

    
    def start(self: Workspace):
        pass