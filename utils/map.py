import numpy as np
def parse_direction(direction):
  """
  direction {0: Top, 1: Bottom, 2: Left, 3: Right}
  """
  try:
      if type(direction) == str:
          direction = {"top": 0 , "bottom": 1, "left": 2, "right": 3}[direction.lower().strip()]
      else:
          assert(type(direction) == int)
          assert(0 <= direction <= 3)
  except AssertionError as e:
      raise(e)
  return direction

def direction2qr_rotation(direction):
    direction = parse_direction(direction)
    goal_rotations = {0: 0, 1: np.deg2rad(180), 2: np.deg2rad(90), 3: np.deg2rad(-90)}
    return goal_rotations[direction]

class QrMap():
    def __init__(self, connections = None, file = None):
        """All the time the option to be connected to another note through one of four directions"""
        self.__connections = {}
        # # [top, bottom, left, right]
        # 1: [0,2,0,4], 
        # 2: [0,3,0,1], 
        # 3: [0,4,0,2], 
        # 4: [0,1,0,3]}

    def add_connection(self, Q1, dir1, Q2, dir2):
        if Q1 not in self.__connections:
            self.__connections[Q1] = [0,0,0,0]
        if Q2 not in self.__connections:
            self.__connections[Q2] = [0,0,0,0]
        self.__connections[Q1][parse_direction(dir1)] = Q2
        self.__connections[Q2][parse_direction(dir2)] = Q1
     
    def node_exists(self, node_id):
        return node_id in self.__connections

    def get_connection_direction(self, start,other):
        try:
            return [i for i in range(4) if self.__connections[start][i] == other][0]
        except:
            raise KeyError(f"A direct connection between qr codes {start} and {other} wasn't found")
            
    def get_plan(self, start: int, stop: int):
        visited = set() 
        unvisited = set(self.__connections.keys()) 
        best = {start: start}
        cost = {start: 0}
        plan = []
        assert(start in unvisited)
        
        while len(unvisited) > 0:
            curr = min(unvisited, key = lambda x : cost[x] if x in cost else 10e15)
            
            if curr == stop:
                while curr != start:
                    plan.append(curr)
                    curr = best[curr]
                plan.append(start)
                break
                
            for neighbour in self.__connections[curr]:
                distance = 1
                if neighbour == 0:
                    continue
                else:
                  if neighbour not in visited:
                    cost[neighbour] = cost[curr] + distance
                    best[neighbour] = curr
                  elif cost[curr] + distance < cost[neighbour]:
                    cost[neighbour] = cost[curr] + distance
                    best[neighbour] = curr
            unvisited.remove(curr)
            visited.add(curr)
        return plan[::-1]

    @staticmethod
    def load(filename):
        raise(NotImplementedError)
        
if __name__ == '__main__':
    map_ = QrMap()
    
    map_.add_connection(1, "right", 2, "bottom")
    map_.add_connection(2, "left", 3, "bottom")
    map_.add_connection(3, "left", 4, "bottom")
    print(map_.get_plan(1,4))