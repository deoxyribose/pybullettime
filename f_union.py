#!/usr/bin/env python3

import numpy as np
import pybullet as pb
import time
import pybullet_data

rng = np.random.default_rng()

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

def makeThingy(nums, pos, vels, forces):
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


    return assemblyId


def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -10)
    planeID = pb.loadURDF("plane.urdf")

    obId = makeThingy(20, vels = rng.uniform(-2, 2, 20).tolist(), forces = rng.uniform(1, 100, 20).tolist(), pos=[0, 0, 5])
    ob2Id = makeThingy(20, pos=[-5, 0, 5], vels = rng.uniform(-2, 2, 20).tolist(), forces = rng.uniform(1, 100, 20).tolist())

    def setVelsForces(multiplier):
        for i in [obId, ob2Id]:
            vels = rng.uniform(-2, 2, 20).tolist()
            # forces = rng.uniform(1, 100, 20).tolist()
            forces = rng.geometric(0.5, 20).tolist()
            assemblyJoints = pb.getNumJoints(i)
            print(f"Assembly joints: {assemblyJoints}")
            for joint in range(assemblyJoints):
                pb.setJointMotorControl2(i, joint, pb.VELOCITY_CONTROL, targetVelocity=multiplier*vels[joint], force=multiplier*forces[joint])

    sphereCol = pb.createCollisionShape(pb.GEOM_SPHERE, radius=0.05)
    sphereId = pb.createMultiBody(baseMass=100, basePosition=[0, -20, 0],
                                  baseCollisionShapeIndex=sphereCol,
                                  baseVisualShapeIndex=-1)

    pb.setRealTimeSimulation(1)
    firing = True

    frames_passed = 0
    stop = False
    while (True):
        pb.stepSimulation()
        time.sleep(1.0/240.0)
        # every 3 seconds
        frames_passed += 1
        if frames_passed % 500 == 0:
            print(frames_passed)
            if stop:
                setVelsForces(0)
            else:
                setVelsForces(np.random.uniform(0.5, 10.5))
            stop = not stop
        # print(pb.getBasePositionAndOrientation(obId))
        # vel, _ = pb.getBaseVelocity(sphereId)
        # if firing:
        #     pb.applyExternalForce(sphereId, -1, [0, 2000, 500], [0, 0, 0], flags=pb.LINK_FRAME)
        #     print(vel)
        #     if np.linalg.norm(vel) > 32:
        #         firing = False
        # else:
        #     if np.linalg.norm(vel) < 2:
        #         pb.resetBasePositionAndOrientation(sphereId, [0, -20, 0], [0, 0, 0, 1])
        #         pb.resetBaseVelocity(sphereId, [0, 0, 0], [0, 0, 0])
        #         firing = True



if __name__ == "__main__":
    main()
