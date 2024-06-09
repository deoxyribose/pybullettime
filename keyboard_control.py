import pybullet as pb
import numpy as np

def move_arrows():
    # moves the camera along the x and y axis of the world
    keys = pb.getKeyboardEvents()
    cam = pb.getDebugVisualizerCamera()

    

    x_delta = 0
    y_delta = 0
    
    if keys.get(65298):  # DOWN
        y_delta = - 0.125
    if keys.get(65297):   # UP
        y_delta = 0.125
    if keys.get(65296):   # LEFT
        x_delta = 0.125
    if keys.get(65295):  # RIGHT
        x_delta = - 0.125

    camera_delta = [x_delta, y_delta, 0]
    new_world_coords = transform_coordinates(camera_delta, cam)
    pb.resetDebugVisualizerCamera(cameraYaw = cam[8], cameraPitch= cam[9],cameraDistance = cam[10],cameraTargetPosition=new_world_coords) 

def transform_coordinates(camera_delta, cam):
    # rotates the coordinates from the camera to the world
    world_coords = cam[11]

    viewMatrix = cam[2]
    viewMatrix = np.array(viewMatrix).reshape(4, 4)

    rotMatrix = viewMatrix[:3, :3]
    translation = viewMatrix[:3, 3]

    camera_delta = np.array(camera_delta) - translation
    world_delta = rotMatrix @ camera_delta

    world_delta[-1] = 0
    new_coords = world_coords + world_delta
    return new_coords
