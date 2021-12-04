from engine import Engine
from Box2D import (b2World, b2AABB, b2CircleShape, b2Color, b2Vec2)
import numpy as np
import multiprocessing as mp
import time



class Es():

    def new_genome(self):
        self.nodes = np.zeros((16, 16), dtype=np.uint8)
        self.bonds = np.zeros((16, 16, 2, 3), dtype=np.uint8)  # each node can bond left or down
        for x in range(5, 10):
            for y in range(5, 10):
                self.nodes[x, y] = 1
                self.bonds[x, y, 0] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]
                self.bonds[x, y, 1] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]
        return (self.nodes, self.bonds)

    def worker(self, genome_queue, result_queue, worker_id):
        while True:
            # get the new actor's params
            max, genome = genome_queue.get()
            world = b2World(gravity=(0, -10), doSleep=True)
            timeStep = 1.0 / 60
            velocityIterations = 8
            positionIterations = 3
            world.warmStarting = True
            world.continuousPhysics = True
            world.subStepping = False

            nodes = np.copy(genome[0])
            bonds = np.copy(genome[1])
            for x in range(0, 15):
                for y in range(0, 15):
                    if np.random.rand() < 0.05:
                        nodes[x, y] = np.random.rand() < 0.5
                    if np.random.rand() < 0.05:
                        bonds[x, y, 0] = [np.random.rand() < 0.5, np.random.rand() < 0.5, np.random.rand() < 0.5]
                    if np.random.rand() < 0.05:
                        bonds[x, y, 1] = [np.random.rand() < 0.5, np.random.rand() < 0.5, np.random.rand() < 0.5]
            engine = Engine(world, None, None, nodes, bonds)
            for i in range(1000):
                world.Step(timeStep, velocityIterations, positionIterations)
                world.ClearForces()
                engine.Step()
            if not (engine.bodies[6][6] is None):
                dist = engine.bodies[6][6].position.x
                # print(worker_id, max, dist)
                result_queue.put((dist, (nodes, bonds)))
            else:
                result_queue.put((-1, None))


    def __init__(self):
        num_processes = 4

        genome_queue = mp.Queue()
        result_queue = mp.Queue()
        self.max = 0
        self.max_genome = None

        processes = []
        for i in range(num_processes):
            p = mp.Process(target=self.worker, args=(genome_queue, result_queue, i))
            p.start()
            processes.append(p)

        for i in range(num_processes):
            genome_queue.put((0, self.new_genome()))

        while True:
            dist, genome = result_queue.get()
            if dist != -1:
                if dist > self.max:
                    print(dist)
                    self.max = dist
                    self.max_genome = genome
                    with open('nodes.npy', 'wb') as f:
                        np.save(f, genome[0])
                    with open('bonds.npy', 'wb') as f:
                        np.save(f, genome[1])
            genome_queue.put((self.max, self.max_genome))


        # max = 0
        # genome = None
        # for j in range(2000):
        #     self.world = b2World(gravity=(0, -10), doSleep=True)
        #
        #     timeStep = 1.0 / 60
        #     velocityIterations = 8
        #     positionIterations = 3
        #     self.world.warmStarting = True
        #     self.world.continuousPhysics = True
        #     self.world.subStepping = False
        #
        #     nodes = np.copy(self.nodes)
        #     bonds = np.copy(self.bonds)
        #     for x in range(0, 15):
        #         for y in range(0, 15):
        #             if np.random.rand() < 0.01:
        #                 nodes[x, y] = np.random.rand() < 0.5
        #             if np.random.rand() < 0.01:
        #                 bonds[x, y, 0] = [np.random.rand() < 0.5, np.random.rand() < 0.5, np.random.rand() < 0.5]
        #             if np.random.rand() < 0.01:
        #                 bonds[x, y, 1] = [np.random.rand() < 0.5, np.random.rand() < 0.5, np.random.rand() < 0.5]
        #
        #
        #     self.engine = Engine(self.world, None, None, nodes, bonds)
        #     for i in range(1000):
        #         self.world.Step(timeStep, velocityIterations, positionIterations)
        #         self.world.ClearForces()
        #         self.engine.Step()
        #     if self.engine.bodies[6][6] is None:
        #         continue
        #     dist = self.engine.bodies[6][6].position.x
        #     print(dist)
        #     if dist > max:
        #         print("New rec", dist)
        #         max = dist
        #         genome = (self.engine.nodes, self.engine.bonds)
        #         self.nodes = nodes
        #         self.bonds = bonds
        #         with open('nodes.npy', 'wb') as f:
        #             np.save(f, genome[0])
        #         with open('bonds.npy', 'wb') as f:
        #             np.save(f, genome[1])



es = Es()