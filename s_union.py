#!/usr/bin/env python3

import numpy as np
import pybullet as pb
import time
import pybullet_data

rng = np.random.default_rng()

class GunBullet:
    def __init__(self, coll, initialOffset=0):
        self.sphereId = pb.createMultiBody(baseMass=100, basePosition=[0, -20-initialOffset, 0],
                                      baseCollisionShapeIndex=coll,
                                      baseVisualShapeIndex=-1)
        pb.changeDynamics(self.sphereId, -1, restitution=0.0)

        self.firing = True
        self.difference = [0, 20 + initialOffset, 2.5 + initialOffset/2.0]

    def update(self, pos1, pos2):
        vel, _ = pb.getBaseVelocity(self.sphereId)
        if self.firing:
            pb.applyExternalForce(self.sphereId, -1, [2000 * self.difference[0], 2000 * self.difference[1], 2000 * self.difference[2] + 200 * np.linalg.norm(self.difference)], [0, 0, 0], flags=pb.LINK_FRAME)
            if np.linalg.norm(vel) > 2*np.linalg.norm(self.difference):
                self.firing = False
        else:
            if np.linalg.norm(vel) < 2:
                pb.resetBasePositionAndOrientation(self.sphereId, [0, -20, 0], [0, 0, 0, 1])
                pb.resetBaseVelocity(self.sphereId, [0, 0, 0], [0, 0, 0])
                self.firing = True
                if rng.binomial(1, 0.5) > 0:
                    self.difference = np.array(pos1) - np.array([0, -20, 0])
                else:
                    self.difference = np.array(pos2) - np.array([0, -20, 0])


def jointMaker(x):
    if x % 2 == 0:
        return pb.JOINT_REVOLUTE
    else:
        return pb.JOINT_FIXED

def randomParents(nums):
    parents = []
    for index in range(nums):
        parents.append(rng.choice(range(index+1)))

    return parents

def makeThingy(nums, pos=[0, 0, 5]):
    cubeCol = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
    leftCubeCol = pb.createCollisionShape(pb.GEOM_BOX,
                                          halfExtents=[0.5, 0.5, 0.5],
                                          collisionFramePosition=[-0.5, 0, 0.5])

    assemblyId =  pb.createMultiBody(baseMass=1, basePosition=pos,
                                     baseOrientation=[0, 0, 0, 1],
                       baseCollisionShapeIndex=cubeCol,
                              baseVisualShapeIndex=-1,
                              linkMasses=[1.0 / (2.0 ** x) for x in range(nums)],
                              linkCollisionShapeIndices=[leftCubeCol] * nums,
                              linkVisualShapeIndices=[leftCubeCol] * nums,
                                     linkPositions=[[-0.5, 0, 0.5]] + [[-1, 0, 1]] * (nums - 1),
                              linkOrientations=[[0, 0, 0, 1]] * nums,
                              linkInertialFramePositions=[[0, 0, 0]] * nums,
                              linkInertialFrameOrientations=[[0, 0, 0, 1]] * nums,
                                     linkParentIndices=randomParents(nums),
                              # linkParentIndices=range(nums),
                                     linkJointTypes=[pb.JOINT_REVOLUTE] * nums,
                                     # linkJointTypes=[jointMaker(x) for x in range(nums)],
                                     linkJointAxis=rng.uniform(-1, 1, (nums, 3)))

    pb.changeDynamics(assemblyId,
                      -1,
                      spinningFriction=0.001,
                      rollingFriction=0.001,
                      linearDamping=0.0)

    assemblyJoints = pb.getNumJoints(assemblyId)
    print(f"Assembly joints: {assemblyJoints}")
    for joint in range(assemblyJoints):
        vel = rng.uniform(-2, 2)
        force = rng.uniform(1, 100)
        pb.setJointMotorControl2(assemblyId, joint, pb.VELOCITY_CONTROL, targetVelocity=vel, force=force)

    return assemblyId


def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -10)
    planeID = pb.loadURDF("plane.urdf")

    obId = makeThingy(20)
    ob2Id = makeThingy(20, pos=[-5, 0, 5])

    sphereCol = pb.createCollisionShape(pb.GEOM_SPHERE, radius=0.1)
    # sphereId = pb.createMultiBody(baseMass=100, basePosition=[0, -20, 0],
    #                               baseCollisionShapeIndex=sphereCol,
    #                               baseVisualShapeIndex=-1)

    # firing = True
    # difference = [0, 20, 2.5]
    guns = [GunBullet(sphereCol, x) for x in range(5)]

    while (True):
        pb.stepSimulation()
        time.sleep(1.0/240.0)

        pos1, _ = pb.getBasePositionAndOrientation(obId)
        if np.abs(pos1[0]) > 20  or np.abs(pos1[1] - 10) > 20:
            pb.resetBasePositionAndOrientation(obId, [0, 0, 5], [0, 0, 0, 1])

        pos2, _ = pb.getBasePositionAndOrientation(ob2Id)
        if np.abs(pos2[0]) > 20  or np.abs(pos2[1] - 10) > 20:
            pb.resetBasePositionAndOrientation(ob2Id, [0, 0, 5], [0, 0, 0, 1])

        [gun.update(pos1, pos2) for gun in guns]

        # vel, _ = pb.getBaseVelocity(sphereId)
        # if firing:
        #     pb.applyExternalForce(sphereId, -1, [2000 * difference[0], 2000 * difference[1], 2000 * difference[2] + 1000], [0, 0, 0], flags=pb.LINK_FRAME)
        #     if np.linalg.norm(vel) > 32:
        #         firing = False
        # else:
        #     if np.linalg.norm(vel) < 2:
        #         pb.resetBasePositionAndOrientation(sphereId, [0, -20, 0], [0, 0, 0, 1])
        #         pb.resetBaseVelocity(sphereId, [0, 0, 0], [0, 0, 0])
        #         firing = True
        #         if rng.binomial(1, 0.5) > 0:
        #             difference = np.array(pos1) - np.array([0, -20, 0])
        #         else:
        #             difference = np.array(pos2) - np.array([0, -20, 0])



if __name__ == "__main__":
    main()
