
class Scene:
    def __init__(self, timestamp, map, ego_agents, env_agents, ped_agents = [], light = 'g'):
        self.timestamp = timestamp
        self.map = map
        self.ego = ego_agents
        self.env = env_agents
        self.peds = ped_agents
        self.light = light
