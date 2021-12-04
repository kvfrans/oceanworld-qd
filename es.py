from engine import Engine
from Box2D import (b2World, b2AABB, b2CircleShape, b2Color, b2Vec2)
import numpy as np
import multiprocessing as mp
import math
import time

class Es():

    def new_genome(self):
        self.nodes = np.zeros((16, 16), dtype=np.uint8)
        self.bonds = np.zeros((16, 16, 2, 3), dtype=np.uint8)  # each node can bond left or down
        for x in range(5, 10):
            for y in range(5, 10):
                self.nodes[x, y] = np.random.rand() < 0.8
                self.bonds[x, y, 0] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]
                self.bonds[x, y, 1] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]
        return (self.nodes, self.bonds)

    def worker(self, genome_queue, result_queue, worker_id):
        while True:
            # get the new actor's params
            genome = genome_queue.get()
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

            if not (engine.bodies[6][6] is None):
                start_y = engine.bodies[6][6].position.y
                for i in range(500):
                    world.Step(timeStep, velocityIterations, positionIterations)
                    world.ClearForces()
                    engine.Step()
                dist = engine.bodies[6][6].position.x - abs(engine.bodies[6][6].position.y - start_y)
                result_queue.put((dist, (nodes, bonds)))
            else:
                result_queue.put((-1, None))


    def __init__(self):
        num_processes = 4
        count = 0

        genome_queue = mp.Queue()
        result_queue = mp.Queue()
        self.max = 0
        self.max_genome = self.new_genome()

        for i in range(num_processes):
            genome_queue.put(self.new_genome())

        processes = []
        for i in range(num_processes):
            p = mp.Process(target=self.worker, args=(genome_queue, result_queue, i))
            p.start()
            processes.append(p)

        while True:
            dist, genome = result_queue.get()
            if dist != -1:
                if dist > self.max:
                    count += 1
                    print(dist)
                    self.max = dist
                    self.max_genome = genome
                    with open('nodes'+str(count)+'.npy', 'wb') as f:
                        np.save(f, genome[0])
                    with open('bonds'+str(count)+'.npy', 'wb') as f:
                        np.save(f, genome[1])
            genome_queue.put(self.max_genome)




es = Es()