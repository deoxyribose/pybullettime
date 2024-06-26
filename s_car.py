#!/usr/bin/env python3

import numpy as np
import pybullet as pb
import time
import pybullet_data
from keyboard_control import move_arrows

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
                                   linkJointTypes=[pb.JOINT_FIXED] * 2 + [pb.JOINT_REVOLUTE] * 2 + [pb.JOINT_FIXED] * 2 + [pb.JOINT_PRISMATIC] * 4 + [pb.JOINT_REVOLUTE] * 4,
                                        linkJointAxis=[[0, 0, 1]] * 10 + [[0, 0, 1]] * 4)

        self.shocks = [ShockAbsorber(1.5 * 2**4, 2) for _ in range(4)]
        for s, l in zip(self.shocks, [2, 5, 9, 12]):
            s.register(self.carId, l-1, l, [0, 0, 1])

        pb.setJointMotorControl2(self.carId, 1, pb.POSITION_CONTROL, targetPosition=0, force=100)

        self.speed(6)


    def speed(self, target_speed):
        pb.setJointMotorControl2(self.carId, 3, pb.VELOCITY_CONTROL, targetVelocity=target_speed, force=10)
        pb.setJointMotorControl2(self.carId, 6, pb.VELOCITY_CONTROL, targetVelocity=target_speed, force=10)
        pb.setJointMotorControl2(self.carId, 7, pb.VELOCITY_CONTROL, targetVelocity=target_speed, force=10)
        pb.setJointMotorControl2(self.carId, 10, pb.VELOCITY_CONTROL, targetVelocity=target_speed, force=10)

    def steer(self, correction):
        if correction > 0.5:
            correction = 0.5
        elif correction < -0.5:
            correction = -0.5

        pb.setJointMotorControl2(self.carId, 1, pb.POSITION_CONTROL, targetPosition=correction, force=100)
        pb.setJointMotorControl2(self.carId, 4, pb.POSITION_CONTROL, targetPosition=correction, force=100)




    def update(self):
        for s in self.shocks:
            s.update()


def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -10)
    planeID = pb.loadURDF("plane.urdf")

    car = Car()

    bump = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.0625, 8, 0.0625])
    bumpLIds = []
    bumpRIds = []
    for ix in range(4):
        bumpLIds.append(pb.createMultiBody(baseMass=100, basePosition=[-4 - 2 * ix, -7.5, 0.2], baseOrientation=[0, 0, 0, 1], baseCollisionShapeIndex=bump, baseVisualShapeIndex=-1))
        bumpRIds.append(pb.createMultiBody(baseMass=100, basePosition=[-5 - 2 * ix, 7.5, 0.2], baseOrientation=[0, 0, 0, 1], baseCollisionShapeIndex=bump, baseVisualShapeIndex=-1))
        last_distance = -5 - 2 * ix

    bumpCIds = []
    for ix in range(8):
        bumpCIds.append(pb.createMultiBody(baseMass=100, basePosition=[last_distance - 1 - ix, 0, 0.2], baseOrientation=[0, 0, 0, 1], baseCollisionShapeIndex=bump, baseVisualShapeIndex=-1))

    while True:
        pb.stepSimulation()
        time.sleep(1.0/240.0)

        target_courses =  []
        pos, ori = pb.getBasePositionAndOrientation(car.carId)
        if pos[0] < -21:
            target_courses.append([0, 0, 3.141593])
            car.speed(18)
        elif pos[0] > 10:
            target_courses.append([0, 0, 0])
            car.speed(6)

        if pos[1] < -22:
            target_courses.append([0, 0, -1.5])
        elif pos[1] > 22:
            target_courses.append([0, 0, 1.5])

        if len(target_courses) > 0:
            target_course = sum(x[2] for x in target_courses) / len(target_courses)
            target_course = pb.getQuaternionFromEuler([0, 0, target_course])
            course = pb.getEulerFromQuaternion(pb.getDifferenceQuaternion(target_course, ori))[2]
        else:
            course = 0
        car.steer(-course)

        car.update()
        move_arrows()

if __name__ == "__main__":
    main()
