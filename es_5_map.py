from engine import Engine
from Box2D import (b2World, b2AABB, b2CircleShape, b2Color, b2Vec2)
import numpy as np
import multiprocessing as mp
import time
import matplotlib.pyplot as plt
# import seaborn as sns
# sns.set_theme()

class Es():

    def new_genome(self):
        self.nodes = np.zeros((16, 16), dtype=np.uint8)
        self.bonds = np.zeros((16, 16, 2, 3), dtype=np.uint8)  # each node can bond left or down
        # for x in range(0, 15):
        #     for y in range(0, 15):
        for x in range(5, 10):
            for y in range(5, 10):
                self.nodes[x, y] = 1
                self.bonds[x, y, 0] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]
                self.bonds[x, y, 1] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]
        return (self.nodes, self.bonds)

    def get_properties(self, genome):
        nodes, bonds = genome
        body_connected = []
        for x in range(16):
            body_connected.append([])
            for y in range(16):
                body_connected[x].append(False)

        def recurse(x, y):
            if body_connected[x][y] == False and nodes[x,y] == 1:
                body_connected[x][y] = True;
                if x > 0 and bonds[x-1, y, 0, 0] == 1:
                    recurse(x-1, y)
                if y > 0 and bonds[x, y-1, 1, 0] == 1:
                    recurse(x, y-1)
                if x < 15 and bonds[x, y, 0, 0] == 1:
                    recurse(x+1, y)
                if y < 15 and bonds[x, y, 1, 0] == 1:
                    recurse(x, y+1)
        recurse(6,6)

        node_count = 0
        bond_count = 1
        rigid_count = 0

        for x in range(16):
            for y in range(16):
                if nodes[x, y] == 1 and body_connected[x][y]:
                    node_count += 1
                if bonds[x,y,0,0] == 1 and x < 15:
                    if nodes[x, y] == 1 and body_connected[x][y] and nodes[x+1, y] == 1 and body_connected[x+1][y]:
                        bond_count += 1
                        if bonds[x,y,0,2] == 1:
                            rigid_count += 1
                if bonds[x, y, 1, 0] == 1 and x < 15:
                    if nodes[x, y] == 1 and body_connected[x][y] and nodes[x, y+1] == 1 and body_connected[x][y+1]:
                        bond_count += 1
                        if bonds[x, y, 1, 2] == 1:
                            rigid_count += 1
        # print("nodecount", node_count, "bondcount", bond_count)
        return node_count, bond_count, rigid_count

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
                        nodes[x, y] = np.random.rand() < 0.8
                    # if np.random.rand() < 0.05:
                    #     nodes[x, y] = 1
                    if np.random.rand() < 0.05:
                        bonds[x, y, 0, 0] = np.random.rand() < 0.5
                    if np.random.rand() < 0.05:
                        bonds[x, y, 0, 1] = np.random.rand() < 0.5
                    if np.random.rand() < 0.05:
                        bonds[x, y, 0, 2] = np.random.rand() < 0.5
                    if np.random.rand() < 0.05:
                        bonds[x, y, 1, 0] = np.random.rand() < 0.5
                    if np.random.rand() < 0.05:
                        bonds[x, y, 1, 1] = np.random.rand() < 0.5
                    if np.random.rand() < 0.05:
                        bonds[x, y, 1, 2] = np.random.rand() < 0.5
            engine = Engine(world, None, None, nodes, bonds)


            if not (engine.bodies[6][6] is None):
                start_y = engine.bodies[6][6].position.y
                for i in range(500):
                    world.Step(timeStep, velocityIterations, positionIterations)
                    world.ClearForces()
                    engine.Step()
                dist = engine.bodies[6][6].position.x - abs(engine.bodies[6][6].position.y - start_y)
                n, b, r = self.get_properties(genome)
                # print("nodecount:", min(n / 100, 1), "bond ratio", b / n - 1)
                result_queue.put((dist, min(n / 100, 1), b / n - 1, (nodes, bonds)))
            else:
                result_queue.put((-1, -1, -1, None))


    def __init__(self):
        num_processes = 16

        genome_queue = mp.Queue()
        result_queue = mp.Queue()
        self.map = np.ones((32,32)) * -1000
        # self.map_genome = [[None] * 32] * 32
        self.map_nodes = np.zeros((32, 32, 16, 16), dtype=np.uint8)
        self.map_bonds = np.zeros((32, 32, 16, 16, 2, 3), dtype=np.uint8)

        processes = []
        for i in range(num_processes):
            p = mp.Process(target=self.worker, args=(genome_queue, result_queue, i))
            p.start()
            processes.append(p)

        with open('nodelist.npy', 'rb') as f:
            nodes = np.load(f)
        with open('bondlist.npy', 'rb') as f:
            bonds = np.load(f)
        for i in range(512):
            genome_queue.put((np.copy(nodes[i]), np.copy(bonds[i])))

        # for i in range(num_processes):
        #     genome_queue.put(self.new_genome())

        count = 0
        while True:
            dist, numNodes, percentRigid, genome = result_queue.get()
            if dist != -1:
                if dist > self.map[min(int(numNodes*32), 31), min(int(percentRigid*32),31)]:
                    print("maploc", min(int(numNodes * 32), 31), min(int(percentRigid * 32), 31), dist)
                    self.map[min(int(numNodes*32), 31), min(int(percentRigid*32),31)] = dist
                    self.map_nodes[min(int(numNodes*32), 31)][min(int(percentRigid*32),31)] = genome[0]
                    self.map_bonds[min(int(numNodes * 32), 31)][min(int(percentRigid * 32), 31)] = genome[1]

            next_g = None
            while True:
                x = np.random.randint(0,32)
                y = np.random.randint(0,32)
                if self.map[x,y] > -1000:
                    next_g = (self.map_nodes[x][y], self.map_bonds[x][y])
                    break
            genome_queue.put(next_g)

            if count % 1000 == 0:
                print(count)
                m = np.copy(self.map).transpose()
                # m = np.flip(m, 0)
                plt.matshow(m, vmin=0, vmax=200)
                plt.xlabel("Number of nodes")
                plt.ylabel("Bond/node ratio")
                # plt.gca().invert_xaxis()
                plt.gca().invert_yaxis()
                plt.savefig("results/es_5_map/figs/"+str(count)+".png")
                with open('results/es_5_map/map_nodes.npy', 'wb') as f:
                    np.save(f, self.map_nodes)
                with open('results/es_5_map/map_bonds.npy', 'wb') as f:
                    np.save(f, self.map_bonds)
                plt.clf()
                # plt.show()
            count += 1
            # print(count)




es = Es()