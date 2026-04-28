from swarm_core import Swarm, SpatialModel, Drone, Base, ResourceNode, Obstacle


def create_mining_swarm() -> Swarm:
    drones = [
        Drone(id=1, pos=(2, 2),  velocity=1),
        Drone(id=2, pos=(3, 5),  velocity=1),
        Drone(id=3, pos=(8, 3),  velocity=1),
        Drone(id=4, pos=(5, 8),  velocity=1),
        Drone(id=5, pos=(1, 10), velocity=1),
    ]
    base = Base(pos=(0, 0))
    resources = [
        ResourceNode(pos=(10, 10), remaining=8),
        ResourceNode(pos=(15, 5),  remaining=12),
        ResourceNode(pos=(5, 15),  remaining=7),
    ]
    obstacles = [
        Obstacle(pos=(7, 7)),
        Obstacle(pos=(12, 12)),
    ]
    model = SpatialModel(drones, base, resources, obstacles)
    return Swarm(model)
