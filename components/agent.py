class Agent:
    def __init__(self, name, x, y, v, goal, orientation, turn=False):
        self.name = name
        self.x = x
        self.y = y
        self.v = v
        self.goal = goal
        self.orientation = orientation
        self.turn = turn
    #
    def step_forward(self):
        self.x = self.x + 1

    def step_mergeL(self):
        self.x = self.x + 1
        self.y = self.y + 1

    def step_mergeR(self):
        self.x = self.x + 1
        self.y = self.y - 1

    def step_stay(self):
        pass

    # def setxy(self,x,y):
    #     self.x = x
    #     self.y = y
