import numpy as np
import pybullet as pb
import time
import pybullet_data

def main():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -1)
    planeID = pb.loadURDF("plane.urdf")
    # objID = pb.loadURDF("model.urdf", [0, 0, 1])
    objID = pb.loadURDF("pybullettime.urdf", [0, 0, 1])
    pb.changeVisualShape(objID, -1, rgbaColor=[1, 1, 1, 1])  # Set color (optional)
    pb.setCollisionFilterGroupMask(objID, -1, 0, 0)  # Disable collisions with everything
    pb.changeDynamics(objID, -1, mass=1.0)  # Set mass
    pb.changeDynamics(objID, -1, lateralFriction=0.5)  # Set lateral friction
    # objID = pb.loadURDF("r2d2.urdf", [1., 0.0, 0.8])

    pb.setRealTimeSimulation(1)
    
    while (True):
        pb.stepSimulation()
        time.sleep(1.0/240.0)

        keys = pb.getKeyboardEvents()
        cam = pb.getDebugVisualizerCamera()
        #Keys to change camera
        if keys.get(100):  #D
            xyz = cam[11]
            x= float(xyz[0]) + 0.125
            y = xyz[1]
            z = xyz[2]
            pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z])
        if keys.get(97):   #A
            xyz = cam[11]
            x= float(xyz[0]) - 0.125
            y = xyz[1]
            z = xyz[2]
            pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z])
        if keys.get(99):   #C
            xyz = cam[11]
            x = xyz[0] 
            y = float(xyz[1]) + 0.125
            z = xyz[2]
            pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z])
        if keys.get(102):  #F
            xyz = cam[11]
            x = xyz[0] 
            y = float(xyz[1]) - 0.125
            z = xyz[2]
            pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z]) 

if __name__ == "__main__":
    main()
