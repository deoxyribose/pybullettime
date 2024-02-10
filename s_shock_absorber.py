#!/usr/bin/env python3

import numpy as np
import pybullet as pb
import time
import pybullet_data

rng = np.random.default_rng()


springConstant = 2 ** 6


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

    pb.setJointMotorControl2(shockId, 0, pb.VELOCITY_CONTROL, targetVelocity=0, force=2)
    # pb.setJointMotorControl2(shockId, 0, pb.VELOCITY_CONTROL, targetVelocity=0, force=8.95)

    sphereCol = pb.createCollisionShape(pb.GEOM_SPHERE)
    spereId = pb.createMultiBody(baseMass=1.25, basePosition=[0, 0, 5], baseCollisionShapeIndex=sphereCol, baseVisualShapeIndex=-1)


    while True:
        pb.stepSimulation()
        time.sleep(1.0/240.0)

        shockState = pb.getJointState(shockId, 0)
        springForce = -springConstant * shockState[0]
        pb.applyExternalForce(shockId, 0, [0, 0, springForce], [0, 0, 0], pb.LINK_FRAME)
        pb.applyExternalForce(shockId, -1, [0, 0, -springForce], [0, 0, 0], pb.LINK_FRAME)
        # update shock joint forces


if __name__ == "__main__":
    main()
