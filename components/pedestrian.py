class Pedestrian:
    def __init__(self, name, x, y, cwloc, goal):
        self.name = name
        self.x = x
        self.y = y
        self.goal = goal
        self.cwloc = cwloc
    #
    def step_forward(self):
        self.cwloc = self.cwloc + 1

    def step_backward(self):
        self.cwloc = self.cwloc - 1

    def update_cell(self, crosswalk):
        self.cell = crosswalk[self.cwloc]

    def step_stay(self):
        pass
