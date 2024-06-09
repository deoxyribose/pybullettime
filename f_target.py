#!/usr/bin/env python3

import numpy as np
import pybullet as pb
import time
import pybullet_data
from keyboard_control import move_arrows

class Target():
    def __init__(self):
        disc = pb.createCollisionShape(pb.GEOM_CYLINDER, radius=2.5, height=0.2)
        small_disc = pb.createCollisionShape(pb.GEOM_CYLINDER, radius=0.5, height=0.3)
        self.id = pb.createMultiBody(baseMass=0,
                                     basePosition=[0, 0, 4.5],
                                     baseOrientation=[0, 1, 0, 1],
                                     baseCollisionShapeIndex=disc,
                                     baseVisualShapeIndex=disc,
                                     linkMasses=[0.5, 0.5],
                                     linkCollisionShapeIndices=[disc, small_disc],
                                     linkVisualShapeIndices=[disc, small_disc],
                                     linkPositions=[[0, 0, 0], [0, 0, 0]],
                                     linkOrientations=[[0, 0, 0, 1], [1, 0, 0, 0]],
                                     linkParentIndices=[0, 0],
                                     linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
                                     linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
                                     linkJointTypes=[pb.JOINT_FIXED, pb.JOINT_FIXED],
                                     linkJointAxis=[[0, 0, 1], [0, 0, 1]]
        )

class Bullet():
    def __init__(self):
        bullet = pb.createCollisionShape(pb.GEOM_SPHERE, radius=0.5)
        self.id = pb.createMultiBody(baseMass=0.1,
                                     basePosition=[10, 0, 0.5],
                                     baseOrientation=[0, 0, 0, 1],
                                     baseCollisionShapeIndex=bullet,
                                     baseVisualShapeIndex=bullet,
                                    #  linkMasses=[0.1],
                                    #  linkCollisionShapeIndices=[bullet],
                                    #  linkVisualShapeIndices=[bullet],
                                    #  linkPositions=[[0, 0, 0]],
                                    #  linkOrientations=[[0, 0, 0, 1]],
                                    #  linkParentIndices=[0],
                                    #  linkInertialFramePositions=[[0, 0, 0]],
                                    #  linkInertialFrameOrientations=[[0, 0, 0, 1]],
                                    #  linkJointTypes=[pb.JOINT_FIXED],
                                    #  linkJointAxis=[[0, 0, 1]]
        )
        
def computeInitForce():
    # we want to move the ball to the target
    # the target is at [0, 0, 4.5]
    # the ball is at [10, 0, 0.5]
    # the ball has mass 0.1
    # the ball's initial velocity is 0
    # we want to move this mass to the target in 1 second
    # taking into account gravity
    
    # force = mass * acceleration
    # acceleration = (final_velocity - initial_velocity) / time
    # force = mass * (final_velocity - initial_velocity) / time

    # final_velocity = displacement / time
    # displacement = target_position - initial_position
    # final_velocity = (target_position - initial_position) / time

    # force = mass * ((target_position - initial_position) / time - initial_velocity) / time
    # force = mass * ((target_position - initial_position) / time - 0) / time

    # force = mass * (target_position - initial_position) / time**2
    force = 0.1 * np.array([10, 0, 4.0]) / 1**2
    return force * 100


def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.86)
    planeID = pb.loadURDF("plane.urdf")

    # set initial camera
    cam = pb.getDebugVisualizerCamera()
    pb.resetDebugVisualizerCamera(cameraDistance=cam[10], cameraYaw=cam[8], cameraPitch=cam[9], cameraTargetPosition=[15, -3, 0])

    target = Target()
    bullet = Bullet()

    initForce = computeInitForce()
    print(initForce)
    time.sleep(.3)
    start = time.time()
    pb.applyExternalForce(bullet.id, -1, forceObj=initForce, posObj=[15, 0, 0.5], flags=pb.LINK_FRAME)

    while True:
        move_arrows()
        pb.stepSimulation()
        time.sleep(1/240)

        # detect collision between bullet and target
        contact = pb.getContactPoints(bullet.id, target.id)
        if contact:
            print("Hit!")
            print(time.time() - start)
            # break

if __name__ == "__main__":
    main()
