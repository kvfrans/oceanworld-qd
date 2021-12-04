from engine import Engine
from Box2D import (b2World, b2AABB, b2CircleShape, b2Color, b2Vec2)
import numpy as np


class Es():
    def __init__(self):

        self.nodes = np.zeros((16, 16), dtype=np.uint8)
        self.bonds = np.zeros((16, 16, 2, 3), dtype=np.uint8)  # each node can bond left or down
        for x in range(5, 10):
            for y in range(5, 10):
                self.nodes[x, y] = 1
                self.bonds[x, y, 0] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]
                self.bonds[x, y, 1] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]

        max = 0
        genome = None
        for j in range(2000):
            self.world = b2World(gravity=(0, -10), doSleep=True)

            timeStep = 1.0 / 60
            velocityIterations = 8
            positionIterations = 3
            self.world.warmStarting = True
            self.world.continuousPhysics = True
            self.world.subStepping = False

            nodes = np.copy(self.nodes)
            bonds = np.copy(self.bonds)
            for x in range(0, 15):
                for y in range(0, 15):
                    if np.random.rand() < 0.01:
                        nodes[x, y] = np.random.rand() < 0.5
                    if np.random.rand() < 0.01:
                        bonds[x, y, 0] = [np.random.rand() < 0.5, np.random.rand() < 0.5, np.random.rand() < 0.5]
                    if np.random.rand() < 0.01:
                        bonds[x, y, 1] = [np.random.rand() < 0.5, np.random.rand() < 0.5, np.random.rand() < 0.5]


            self.engine = Engine(self.world, None, None, nodes, bonds)
            for i in range(1000):
                self.world.Step(timeStep, velocityIterations, positionIterations)
                self.world.ClearForces()
                self.engine.Step()
            if self.engine.bodies[6][6] is None:
                continue
            dist = self.engine.bodies[6][6].position.x
            print(dist)
            if dist > max:
                print("New rec", dist)
                max = dist
                genome = (self.engine.nodes, self.engine.bonds)
                self.nodes = nodes
                self.bonds = bonds
                with open('nodes.npy', 'wb') as f:
                    np.save(f, genome[0])
                with open('bonds.npy', 'wb') as f:
                    np.save(f, genome[1])



es = Es()