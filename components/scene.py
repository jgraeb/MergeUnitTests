
class Scene:
    def __init__(self, timestamp, map, ego_agents, env_agents, ped_agents = []):
        self.timestamp = timestamp
        self.map = map
        self.ego = ego_agents
        self.env = env_agents
        self.peds = ped_agents
