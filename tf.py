class Pose():
    def __init__(self,x=0,y=0,rot=0):
        self.x = x
        self.y = y
        self.rot = rot

    def __add__(self, other):
        out = Pose()
        out.x = self.x + other.x
        out.y = self.y + other.y
        out.rot = (self.rot + other.rot) % 360 # ! It should probably be in radians
        return out

    def __sub__(self, other):
        out = Pose()
        out.x = self.x - other.x
        out.y = self.y - other.y
        out.rot = (self.rot - other.rot) % 360 # ! It should probably be in radians
        return out
    def __repr__(self):
        return f"X: {self.x}, Y: {self.y}, ROT: {self.rot}:"
class TFManager():
    def __init__(self):
        self.__frames = set()
        self.__tfs = dict()
        
    def create_frame(self, frame_name):
        self.__frames.add(frame_name)

    def init_tf(self, parent_frame, child_frame):
        if parent_frame not in self.__frames:
            pass
        if child_frame not in self.__frames:
            pass
        if parent_frame not in self.__tfs:
            pass
        if child_frame not in self.__tfs:
            pass
        self.__tfs[parent_frame][child_frame].add(Pose())

    def set_tf(self, pose, parent_frame, child_frame): # ! Should require that the frames are already directly connected
        self.__tfs[parent_frame][child_frame] = pose
        self.__tfs[child_frame][parent_frame] = Pose() - pose
        
    def get_tf(self, parent_frame, child_frame): 
        return self.__tfs[parent_frame][child_frame]