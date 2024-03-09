#!/usr/bin/env python3

import numpy as np
import pybullet as pb
import time
import pybullet_data

from s_shock_absorber import ShockAbsorber

class Car():
    def __init__(self):
        hoodCol = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[1, 1, 0.25])
        cabinCol = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[1, 1, 0.75])
        trunkCol = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.5, 1, 0.25])

        shockCol = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])

        wheelCol = pb.createCollisionShape(pb.GEOM_CYLINDER, radius=0.5, height=0.2)

        self.carId = pb.createMultiBody(baseMass=1,
                                   basePosition=[0, 0, 2],
                                   baseOrientation=[0, 0, 0, 1],
                                   baseCollisionShapeIndex=cabinCol,
                                   baseVisualShapeIndex=-1,
                                   linkMasses=[2, 1] + [0.02] * 8 + [0.04] * 4,
                                   linkCollisionShapeIndices=[hoodCol, trunkCol] + [shockCol] * 8 + [wheelCol] * 4,
                                   linkVisualShapeIndices=[hoodCol, trunkCol] + [shockCol] * 8 + [wheelCol] * 4,
                                   linkPositions=[[-2, 0, -0.5], [1.5, 0, -0.5],
                                                  [-0.5, -1, -0.15], [-0.5, 1, -0.15],
                                                  [0, -1, -0.15], [0, 1, -0.15]]
                                   + [[0, 0, -1]] * 4
                                        + [[0, -0.1, 0], [0, 0.1, 0], [0, -0.1, 0], [0, 0.1, 0]],
                                   linkOrientations=[[0, 0, 0, 1]] * 10 + [[1, 0, 0, 1]] * 4,
                                   linkParentIndices=[0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10],
                                   linkInertialFramePositions=[[0, 0, 0]] * 14,
                                   linkInertialFrameOrientations=[[0, 0, 0, 1]] * 14,
                                   linkJointTypes=[pb.JOINT_FIXED] * 6 + [pb.JOINT_PRISMATIC] * 4 + [pb.JOINT_REVOLUTE] * 4,
                                        linkJointAxis=[[0, 0, 1]] * 10 + [[0, 0, 1]] * 4)

        self.shocks = [ShockAbsorber(1.5 * 2**4, 2) for _ in range(4)]
        for s, l in zip(self.shocks, [2, 5, 9, 12]):
            s.register(self.carId, l-1, l, [0, 0, 1])

        pb.setJointMotorControl2(self.carId, 3, pb.VELOCITY_CONTROL, targetVelocity=4, force=10)
        pb.setJointMotorControl2(self.carId, 6, pb.VELOCITY_CONTROL, targetVelocity=4, force=10)
        pb.setJointMotorControl2(self.carId, 7, pb.VELOCITY_CONTROL, targetVelocity=4, force=10)
        pb.setJointMotorControl2(self.carId, 10, pb.VELOCITY_CONTROL, targetVelocity=4, force=10)

    def update(self):
        for s in self.shocks:
            s.update()


def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -10)
    planeID = pb.loadURDF("plane.urdf")

    car = Car()

    while True:
        pb.stepSimulation()
        time.sleep(1.0/240.0)

        car.update()

if __name__ == "__main__":
    main()
