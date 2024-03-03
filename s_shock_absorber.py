#!/usr/bin/env python3

import numpy as np
import pybullet as pb
import time
import pybullet_data

rng = np.random.default_rng()


springConstant = 2 ** 6


class ShockAbsorber:
    def __init__(self, springConstant, dampingForce):
        self.springConstant = springConstant
        self.dampingForce = dampingForce
        self.registered = False

    def register(self, shockId, linkParent, linkChild, linkAxis):
        self.shockId = shockId
        self.linkParent = linkParent
        self.linkChild = linkChild
        self.linkAxis = linkAxis
        pb.setJointMotorControl2(self.shockId,
                                 self.linkChild,
                                 pb.VELOCITY_CONTROL,
                                 targetVelocity=0,
                                 force=self.dampingForce)
        self.registered = True

    def update(self):
        if not self.registered:
            raise RuntimeError("You need to register before you can simulate")

        shockState = pb.getJointState(self.shockId, self.linkChild)
        springForce = -self.springConstant * shockState[0]

        pb.applyExternalForce(self.shockId, self.linkParent,
                              [-springForce * x for x in self.linkAxis], [0, 0, 0], pb.LINK_FRAME)
        pb.applyExternalForce(self.shockId, self.linkChild,
                              [springForce * x for x in self.linkAxis], [0, 0, 0], pb.LINK_FRAME)

def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -10)
    planeID = pb.loadURDF("plane.urdf")

    cubeCol = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])

    shockId = pb.createMultiBody(baseMass=1, basePosition=[0, 0, 1], baseOrientation=[0, 0, 0, 1],
                                 baseCollisionShapeIndex=cubeCol, baseVisualShapeIndex=-1, linkMasses=[1],
                                 linkCollisionShapeIndices=[cubeCol], linkVisualShapeIndices=[cubeCol],
                                 linkPositions=[[0, 0, 2]], linkOrientations=[[0, 0, 0, 1]],
                                 linkParentIndices=[0],
                                 linkInertialFramePositions=[[0, 0, 0]],
                                 linkInertialFrameOrientations=[[0, 0, 0, 1]],
                                 linkJointTypes=[pb.JOINT_PRISMATIC],
                                 linkJointAxis=[[0, 0, 1]])

    sphereCol = pb.createCollisionShape(pb.GEOM_SPHERE)
    spereId = pb.createMultiBody(baseMass=1.25, basePosition=[0, 0, 5], baseCollisionShapeIndex=sphereCol, baseVisualShapeIndex=-1)

    sp = ShockAbsorber(springConstant, 2)
    sp.register(shockId, -1, 0, [0, 0, 1])

    while True:
        pb.stepSimulation()
        time.sleep(1.0/240.0)

        # update shock joint forces
        sp.update()

if __name__ == "__main__":
    main()
