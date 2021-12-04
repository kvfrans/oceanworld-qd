from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2DistanceJointDef, b2EdgeShape, b2FixtureDef,
                   b2PolygonShape, b2CircleShape, b2RevoluteJointDef, b2PrismaticJointDef, b2Color)
import math
import numpy as np
from engine import Engine
import sys

nl = "nodes.npy"
bl = "bonds.npy"
xp = 0
yp = 0

class Web(Framework):
    name = ""
    description = ""
    bodies = []
    joints = []

    def __init__(self):
        super(Web, self).__init__()
        with open(nl, 'rb') as f:
            nodes = np.load(f)
        with open(bl, 'rb') as f:
            bonds = np.load(f)

        if 'map' in nl:
            nodes = nodes[xp,yp]
            bonds = bonds[xp,yp]

        # nodes = np.zeros((16, 16), dtype=np.uint8)
        # bonds = np.zeros((16, 16, 2, 3), dtype=np.uint8)  # each node can bond left or down
        # for x in range(5, 10):
        #     for y in range(5, 10):
        #         nodes[x, y] = np.random.rand() < 0.8
        #         bonds[x, y, 0] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]
        #         bonds[x, y, 1] = [1, np.random.rand() < 0.5, np.random.rand() < 0.5]

        self.eng = Engine(self.world, self.renderer, self.colors, nodes, bonds)
        self.viewZoom = 4
        self.viewCenter = (37, 37.03)
        self.settings.drawMenu = False
        self.settings.drawStats = False
        self.settings.drawFPS = False

    def Step(self, settings):
        self.renderer.DrawSolidCircle((0,0), 2000, (0,0), b2Color(0.4, 0.4, 0.5))
        super().Step(settings)
        self.eng.Step()
        # print(self.viewZoom)
        print(self.viewCenter)

if __name__ == "__main__":
    if(len(sys.argv) == 1):
        # w = Web()
        main(Web)
    else:
        nl = sys.argv[1]
        bl = sys.argv[2]
        if len(sys.argv) > 3:
            xp = int(sys.argv[3])
            yp = int(sys.argv[4])
        main(Web)
