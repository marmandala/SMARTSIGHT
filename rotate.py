import airsim
import math

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()
global current_rotation
current_rotation = 0

def rotate(rotation_degrees):
    global current_rotation
    target_rotation = current_rotation + rotation_degrees
    client.rotateToYawAsync(target_rotation, 5, 1).join()
    current_rotation = target_rotation

rotation_degrees = 90

airsim.wait_key('Press any key to rotate')
rotate(rotation_degrees)
airsim.wait_key('Press any key to rotate')
rotate(-rotation_degrees)
airsim.wait_key('Press any key to rotate')
rotate(rotation_degrees)
airsim.wait_key('Press any key to rotate')
rotate(-rotation_degrees)

client.reset()
client.armDisarm(False)
client.enableApiControl(False)

