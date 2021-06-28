class Pose():
    def __init__(self, x=0, y=0, rot=0):
        self.x = x
        self.y = y
        self.rot = rot

    def __add__(self, other):
        out = Pose()
        out.x = self.x + other.x
        out.y = self.y + other.y
        out.rot = (self.rot + other.rot) % 360
        return out

    def __sub__(self, other):
        out = Pose()
        out.x = self.x - other.x
        out.y = self.y - other.y
        out.rot = (self.rot - other.rot) % 360
        return out

    def __repr__(self):
        return f"X: {self.x}, Y: {self.y}, ROT: {self.rot}"