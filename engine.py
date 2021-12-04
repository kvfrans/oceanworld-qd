from Box2D import (b2DistanceJointDef, b2EdgeShape, b2FixtureDef,
                   b2PolygonShape, b2CircleShape, b2RevoluteJointDef, b2PrismaticJointDef)
import math
import numpy as np


class Engine():
    def __init__(self, world, renderer=None, colors=None, nodes=None, bonds=None):
        # pass in a box2d world and an optional renderer
        self.world = world
        self.renderer = renderer
        self.colors = colors
        self.stepCount = 0

        self.world.gravity = (0,0)
        self.opened = False

        # The ground
        # self.world.CreateBody(shapes=b2EdgeShape(vertices=[(-200, -200), (200, -200)]))
        # self.world.CreateBody(shapes=b2EdgeShape(vertices=[(-200, 200), (200, 200)]))
        # self.world.CreateBody(shapes=b2EdgeShape(vertices=[(200, -200), (200, 200)]))
        # self.world.CreateBody(shapes=b2EdgeShape(vertices=[(-200, -200), (-200, 200)]))

        circle = b2FixtureDef(shape=b2CircleShape(radius=0.5), density=1, friction=0.2)
        square = b2FixtureDef(shape=b2PolygonShape(box=(0.5, 0.5)), density=1, friction=0.2)

        self.nodes = np.zeros((16, 16), dtype=np.uint8)
        self.bonds = np.zeros((16, 16, 2, 3), dtype=np.uint8) # each node can bond left or down

        for x in range(5, 10):
            for y in range(5, 10):
                self.nodes[x,y] = 1
                self.bonds[x, y, 0] = [1, 1 if np.random.rand() < 0.5 else 0, 1 if np.random.rand() < 0.5 else 0]
                self.bonds[x, y, 1] = [1, 1 if np.random.rand() < 0.5 else 0, 1 if np.random.rand() < 0.5 else 0]

        if not (nodes is None):
            self.nodes = nodes
            self.bonds = bonds

        self.body_connected = []
        for x in range(16):
            self.body_connected.append([])
            for y in range(16):
                self.body_connected[x].append(False)

        def recurse(x, y):
            if self.body_connected[x][y] == False and self.nodes[x,y] == 1:
                self.body_connected[x][y] = True;
                if x > 0 and self.bonds[x-1, y, 0, 0] == 1:
                    recurse(x-1, y)
                if y > 0 and self.bonds[x, y-1, 1, 0] == 1:
                    recurse(x, y-1)
                if x < 15 and self.bonds[x, y, 0, 0] == 1:
                    recurse(x+1, y)
                if y < 15 and self.bonds[x, y, 1, 0] == 1:
                    recurse(x, y+1)
        recurse(6,6)

        length = 5
        self.bodies = []
        for x in range(16):
            self.bodies.append([])
            for y in range(16):
                if self.nodes[x,y] == 1 and self.body_connected[x][y]:
                    b = self.world.CreateDynamicBody(position=(x*length, y*length),fixtures=circle)
                    self.bodies[x].append(b)
                else:
                    self.bodies[x].append(None)


        sets = []
        for x in range(16):
            for y in range(16):
                if self.bonds[x,y,0,0] == 1 and x < 15:
                    sets.append(((x,y,self.bodies[x][y]), (x+1,y,self.bodies[x+1][y]), self.bonds[x,y,0,1] == 1, self.bonds[x,y,0,2] == 1))
                if self.bonds[x,y,1,0] == 1 and y < 15:
                    sets.append(((x,y,self.bodies[x][y]), (x,y+1,self.bodies[x][y+1]), self.bonds[x,y,1,1] == 1, self.bonds[x,y,1,2] == 1))

        # We will define the positions in the local body coordinates, the length
        # will automatically be set by the __init__ of the b2DistanceJointDef
        self.distJoints = []
        self.rotJoints = []
        self.edges = []
        self.edgeActive = []
        for bA, bB, is_joint, is_outside in sets:
            x1,y1,bodyA = bA
            x2,y2,bodyB = bB
            if bodyA == None or bodyB == None:
                continue
            self.body_connected[x1][y1] = True
            self.body_connected[x2][y2] = True

            ang = math.atan2(bodyB.position.y - bodyA.position.y, bodyB.position.x - bodyA.position.x) + math.pi*0.5
            delta = (bodyB.position.x - bodyA.position.x, bodyB.position.y - bodyA.position.y)
            center = (bodyA.position.x*0.5 + bodyB.position.x*0.5, bodyA.position.y*0.5 + bodyB.position.y*0.5)

            squareA = self.world.CreateDynamicBody(position=bodyA.position + (delta[0]*0.2, delta[1]*0.2), fixtures=square, angle=ang)
            squareB = self.world.CreateDynamicBody(position=bodyB.position - (delta[0]*0.2, delta[1]*0.2), fixtures=square, angle=ang)
            if is_outside:
                self.edges.append((squareA, squareB))
                self.edgeActive.append(False)

            pfn = b2PrismaticJointDef( bodyA=squareA, bodyB=squareB, anchor=center, axis=delta, lowerTranslation = -0.1, upperTranslation = 0.1, enableLimit = True, enableMotor = True, motorSpeed=0, maxMotorForce=500)
            # pfn = b2PrismaticJointDef( bodyA=squareA, bodyB=squareB, anchor=center, axis=delta, enableMotor = True, motorSpeed=100, maxMotorForce=500)

            j = self.world.CreateJoint(pfn)
            if is_joint:
                self.distJoints.append(j)

            # pfn = b2DistanceJointDef( bodyA=squareA, bodyB=squareB, anchorA=squareA.position, anchorB=squareB.position)
            # self.distJoints.append(self.world.CreateJoint(pfn))
            stretch = 0.33 * math.pi
            rfn = b2RevoluteJointDef( bodyA=bodyA, bodyB=squareA, anchor=bodyA.position, lowerAngle = -stretch, upperAngle = +stretch, enableLimit = True, enableMotor = True, motorSpeed=0, maxMotorTorque=500)
            self.rotJoints.append(self.world.CreateJoint(rfn))
            rfn = b2RevoluteJointDef( bodyA=squareB, bodyB=bodyB, anchor=bodyB.position, lowerAngle = -stretch, upperAngle = +stretch, enableLimit = True, enableMotor = True, motorSpeed=0, maxMotorTorque=500)
            self.rotJoints.append(self.world.CreateJoint(rfn))


        for x in range(16):
            for y in range(16):
                if not (self.bodies[x][y] is None):
                    if not self.body_connected[x][y]:
                        world.DestroyBody(self.bodies[x][y])

    def Step(self):
        self.stepCount += 1
        if self.renderer:
            print(self.stepCount, self.bodies[6][6].position.x)
        # print(self.bodies[6][6].position)
        # auto move
        if self.stepCount % 30 == 0:
            self.opened = not self.opened
            if self.opened:
                for joint in self.distJoints:
                    joint.SetLimits(-0.1, 5.1)
                    joint.motorSpeed = 100
            else:
                for joint in self.distJoints:
                    joint.SetLimits(-0.1, 0.1)
                    joint.motorSpeed = -100
            for i in range(len(self.edgeActive)):
                self.edgeActive[i] = not self.edgeActive[i]

        for i in range(len(self.edges)):
            if self.edgeActive[i]:
                squareA, squareB = self.edges[i]
                velA = squareA.GetLinearVelocityFromLocalPoint((0,0))
                velB = squareB.GetLinearVelocityFromLocalPoint((0, 0))
                # vel = (velA.x*0.5 + velB.x*0.5, velA.y*0.5 + velB.y*0.5)
                surfaceArea = math.sqrt((squareA.position.x - squareB.position.x)**2 + (squareA.position.y - squareB.position.y)**2)
                normal = (squareA.position.y - squareB.position.y, squareB.position.x - squareA.position.x)

                normal = np.array(normal)
                normal /= sum(normal ** 2)

                velA = -np.array(velA) * surfaceArea
                velB = -np.array(velB) * surfaceArea

                dotA = np.dot(velA, normal)
                projA = dotA * normal
                if (dotA > 0) or True:
                    squareA.ApplyForce(projA * 100, squareA.position, True)
                    if self.renderer:
                        self.renderer.DrawSegment(self.renderer.to_screen(squareA.position),
                                             self.renderer.to_screen((squareA.position.x + projA[0], squareA.position.y + projA[1])),
                                             self.colors['bomb_line'])

                dotB = np.dot(velB, normal)
                projB = dotB * normal
                if (dotB > 0) or True:
                    squareB.ApplyForce(projB * 100, squareA.position, True)
                    if self.renderer:
                        self.renderer.DrawSegment(self.renderer.to_screen(squareB.position),
                                             self.renderer.to_screen((squareB.position.x + projB[0], squareB.position.y + projB[1])),
                                             self.colors['bomb_line'])
                if self.renderer:
                    self.renderer.DrawSegment(self.renderer.to_screen(squareA.position),
                                              self.renderer.to_screen(squareB.position),
                                              self.colors['contact_add'])
                    self.renderer.DrawSegment(self.renderer.to_screen(squareA.position),
                                              self.renderer.to_screen((squareA.position.x + normal[0] , squareA.position.y + normal[1])),
                                              self.colors['contact_add'])
                    self.renderer.DrawSegment(self.renderer.to_screen(squareB.position),
                                              self.renderer.to_screen(
                                                  (squareB.position.x + normal[0], squareB.position.y + normal[1])),
                                              self.colors['contact_add'])