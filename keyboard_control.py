import pybullet as pb

def move_arrows():
    keys = pb.getKeyboardEvents()
    cam = pb.getDebugVisualizerCamera()
    print(keys)
    #Keys to change camera
    if keys.get(65298):  # DOWN
        xyz = cam[11]
        x= float(xyz[0]) + 0.125
        y = xyz[1]
        z = xyz[2]
        pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z])
    if keys.get(65297):   # UP
        xyz = cam[11]
        x= float(xyz[0]) - 0.125
        y = xyz[1]
        z = xyz[2]
        pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z])
    if keys.get(65296):   # LEFT
        xyz = cam[11]
        x = xyz[0] 
        y = float(xyz[1]) + 0.125
        z = xyz[2]
        pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z])
    if keys.get(65295):  # RIGHT
        xyz = cam[11]
        x = xyz[0] 
        y = float(xyz[1]) - 0.125
        z = xyz[2]
        pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=[x,y,z]) 