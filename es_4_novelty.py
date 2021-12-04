from engine import Engine
from Box2D import (b2World, b2AABB, b2CircleShape, b2Color, b2Vec2)
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
sns.set_theme()



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
        return node_count, bond_count, rigid_count


    def __init__(self):
        self.population = []
        genome = self.new_genome()
        n, b, r = self.get_properties(genome)

        self.population.append((genome, min(n/100, 1), b / n - 1))

        for i in range(20000):
            x = [p[1] for p in self.population]
            y = [p[2] for p in self.population]
            plt.plot(x, y, 'o', color='gray');

            origins = []
            for j in range(10):
                og = self.new_genome()
                n, b, r = self.get_properties(og)
                if n > 0 and b > 0:
                    origins.append((og, min(n / 100, 1), b / n - 1))
            x = [p[1] for p in origins]
            y = [p[2] for p in origins]
            plt.plot(x, y, 'o', color='red');

            # plt.xlabel("Number of nodes")
            # plt.ylabel("Bond/node ratio")
            # plt.xlim(0, 1);
            # plt.ylim(0, 1);
            # plt.savefig("nov/" + str(i) + ".png")
            # plt.clf()
            # plt.show()

            print(i)
            for j in range(len(self.population)):
                genome = self.population[j][0]
                nodes = np.copy(genome[0])
                bonds = np.copy(genome[1])
                for x in range(0, 15):
                    for y in range(0, 15):
                        if np.random.rand() < 0.05:
                            nodes[x, y] = 1
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
                        # if np.random.rand() < 0.05:
                        #     bonds[x, y, 0] = [np.random.rand() < 0.5, np.random.rand() < 0.5, np.random.rand() < 0.5]
                        # if np.random.rand() < 0.05:
                        #     bonds[x, y, 1] = [np.random.rand() < 0.5, np.random.rand() < 0.5, np.random.rand() < 0.5]
                genome = (nodes, bonds)
                n, b, r = self.get_properties(genome)
                if b > 0 and n > 0:
                    self.population.append((genome, min(n/100, 1), b / n - 1))

            metrics = []
            for j in range(len(self.population)):
                dists = []
                for k in range(len(self.population)):
                    dist = (self.population[j][1] - self.population[k][1])**2 + (self.population[j][2] - self.population[k][2])**2
                    dists.append(dist)
                dists.sort()
                metric = sum(dists[:20])
                metrics.append(metric)

            self.population = [x for _, x in sorted(zip(metrics, self.population), key=lambda pair: pair[0])]
            self.population.reverse()
            self.population = self.population[:512]

            if i % 10 == 0:
                # plt.xlabel("Number of nodes")
                # plt.ylabel("Bond/node ratio")
                # plt.xlim(0, 1);
                # plt.ylim(0, 1);
                # # plt.savefig("nov/" + str(i) + ".png")
                # # plt.clf()
                # plt.show()
                nodelist = np.array([a[0][0] for a in self.population])
                bondlist = np.array([a[0][1] for a in self.population])
                print(np.shape(nodelist))
                print(np.shape(bondlist))
                with open('nodelist.npy', 'wb') as f:
                    np.save(f, nodelist)
                with open('bondlist.npy', 'wb') as f:
                    np.save(f, bondlist)
                # for x in [0, 0.1, 0.3, 0.5, 0.8, 1]:
                #     for y in [0, 0.1, 0.3, 0.5, 0.8, 1]:
                #         metrics = []
                #         for j in range(len(self.population)):
                #             dist = (self.population[j][1] - x) ** 2 + (self.population[j][2] - y) ** 2
                #             metrics.append(dist)
                #
                #         self.population = [x for _, x in sorted(zip(metrics, self.population), key=lambda pair: pair[0])]
                #         genome = self.population[0][0]
                #         with open('nodes'+str(x)+'-'+str(y)+'.npy', 'wb') as f:
                #             np.save(f, genome[0])
                #         with open('bonds'+str(x)+'-'+str(y)+'.npy', 'wb') as f:
                #             np.save(f, genome[1])


es = Es()