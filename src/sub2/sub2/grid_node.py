class grid_node :
    def __init__(self):
        self.y = 0
        self.x = 0
        self.cost = 0

    def __lt__(self, other): 
        return self.cost < other.cost