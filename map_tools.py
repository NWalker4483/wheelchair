
class QrMap():
    def __init__(self):
        """All the time the option to be connected to another note through one of four directions"""
        self.__connections = {
            # [top, bottom, left, right]
            1: [0,3,0,0], 
            2: [0,3,0,0], 
            3: [4,2,0,1], 
            4: [3,0,0,0]}

    def get_connection_direction(self, start,other):
        try:
            return [i for i in range(4) if self.__connections[start][i] == other][0]
        except:
            raise KeyError(f"A connection between qr codes {start} and {other} wasn't found")

    def get_plan(self, start: int, stop: int):
        visited = set() 
        unvisited = set(self.__connections.keys()) 
        best = {start: start}
        cost = {start: 0}
        plan = []
        
        while len(unvisited) > 0:
            curr = min(unvisited, key = lambda x : best[x] if x in best else 10e15)
            if curr == stop:
                while curr != start:
                    plan.append(curr)
                    curr = best[curr]
                plan.append(start)
                break
            for neighbour in self.__connections[curr]:
                distance = 1
                if neighbour > 0:
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
        return QrMap()
if __name__ == '__main__':
    a = QrMap()
    print(a.get_plan(1,4))