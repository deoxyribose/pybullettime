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

class Ball():
    def __init__(self):
        ball = pb.createCollisionShape(pb.GEOM_SPHERE, radius=0.5)
        self.id = pb.createMultiBody(baseMass=0.1,
                                     basePosition=[10, 0, 0.5],
                                     baseOrientation=[0, 0, 0, 1],
                                     baseCollisionShapeIndex=ball,
                                     baseVisualShapeIndex=ball,
                                    #  linkMasses=[0.1],
                                    #  linkCollisionShapeIndices=[ball],
                                    #  linkVisualShapeIndices=[ball],
                                    #  linkPositions=[[0, 0, 0]],
                                    #  linkOrientations=[[0, 0, 0, 1]],
                                    #  linkParentIndices=[0],
                                    #  linkInertialFramePositions=[[0, 0, 0]],
                                    #  linkInertialFrameOrientations=[[0, 0, 0, 1]],
                                    #  linkJointTypes=[pb.JOINT_FIXED],
                                    #  linkJointAxis=[[0, 0, 1]]
        )
        
def computeInitForce():
    return [-200, 0, 300]
    # return [30, 50, 100]

def computeInitPosition(force, ballPos, ball_radius=0.5):
    force_mag = np.linalg.norm(force)
    force_dir = force / force_mag
    return ballPos - force_dir * ball_radius


def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.86)
    planeID = pb.loadURDF("plane.urdf")

    # set initial camera
    cam = pb.getDebugVisualizerCamera()
    pb.resetDebugVisualizerCamera(cameraDistance=cam[10], cameraYaw=cam[8], cameraPitch=cam[9], cameraTargetPosition=[15, -3, 0])

    target = Target()
    ball = Ball()

    initForce = computeInitForce()
    print(initForce)
    start = time.time()
    ballPos, ballOrn = pb.getBasePositionAndOrientation(ball.id)
    print(ballPos)
    # print(ballOrn)

    initPos = computeInitPosition(initForce, ballPos)
    print(initPos)
    pb.applyExternalForce(ball.id, -1, forceObj=initForce, posObj=initPos, flags=pb.WORLD_FRAME)

    time.sleep(1)

    time_elapsed = 0
    while time_elapsed < 10.0:
        move_arrows()
        pb.stepSimulation()
        time.sleep(1/240)

        # detect collision between ball and target
        contact = pb.getContactPoints(ball.id, target.id)
        if contact:
            print("Hit!")
            print(time_elapsed)
            # ballPos, ballOrn = pb.getBasePositionAndOrientation(ball.id)
            # break
        time_elapsed = time.time() - start
    
    pb.disconnect()


if __name__ == "__main__":
    main()
