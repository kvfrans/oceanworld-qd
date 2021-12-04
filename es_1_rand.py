from engine import Engine
from Box2D import (b2World, b2AABB, b2CircleShape, b2Color, b2Vec2)
import numpy as np


class Es():
    def __init__(self):
        max = 1000000
        genome = None
        for j in range(200):
            self.world = b2World(gravity=(0, -10), doSleep=True)
            self.engine = Engine(self.world)
            timeStep = 1.0 / 60
            velocityIterations = 8
            positionIterations = 3
            for i in range(1000):
                self.engine.Step()
                self.world.Step(timeStep, velocityIterations,positionIterations)
                self.world.ClearForces()
            dist = self.engine.bodies[6][6].position.x
            if dist < max:
                print(dist)
                max = dist
                genome = (self.engine.nodes, self.engine.bonds)
                with open('nodes.npy', 'wb') as f:
                    np.save(f, genome[0])
                with open('bonds.npy', 'wb') as f:
                    np.save(f, genome[1])



es = Es()