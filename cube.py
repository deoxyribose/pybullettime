#!/usr/bin/env python3

import pybullet as pb
import time
import pybullet_data

def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -10)
    planeID = pb.loadURDF("plane.urdf")
    startPos = [0.1, 0, 2]
    startOrientation = pb.getQuaternionFromEuler([0, 0, 0])
    astromecId = pb.loadURDF("r2d2.urdf", startPos, startOrientation)
    sphereCol = pb.createCollisionShape(pb.GEOM_SPHERE)
    sphereVis = pb.createVisualShape(pb.GEOM_SPHERE)
    sphereLId = pb.createMultiBody(baseMass=0.2,
                                  baseInertialFramePosition=[0, 0, 0],
                                  basePosition=[0, 0, 1],
                                  baseCollisionShapeIndex=sphereCol,
                                  baseVisualShapeIndex=sphereVis)
    sphereHId = pb.createMultiBody(baseMass=1,
                                  baseInertialFramePosition=[0, 0, 0],
                                  basePosition=[-2, -0.1, 1],
                                  baseCollisionShapeIndex=sphereCol,
                                  baseVisualShapeIndex=sphereVis)
    sphereVLId = pb.createMultiBody(baseMass=0.04,
                                  baseInertialFramePosition=[0, 0, 0],
                                  basePosition=[-1.8, 2, 1],
                                  baseCollisionShapeIndex=sphereCol,
                                  baseVisualShapeIndex=sphereVis)
    for i in range(4000):
        if i < 1000:
            pb.applyExternalForce(sphereHId, -1, [-200, 0, 0], [0, 0, 0], flags=pb.WORLD_FRAME)
        pb.stepSimulation()
        time.sleep(1./240.)
    pb.disconnect()
    print("Huh?")

if __name__ == "__main__":
    main()
