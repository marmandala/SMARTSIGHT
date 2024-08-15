import airsim

# Подключение к симулятору AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

airsim.wait_key('Press any key to move vehicle')

positions = [(10, 4, 0), (10, -4, 0), (10, 4, 0)]

for i in range(0, len(positions)):
    point = positions[i]
    print(f"To {point}")
    pose = client.simGetVehiclePose()
    target_position = pose.position + airsim.Vector3r(point[0]*2, point[1]*2, point[2]*2)
    client.moveToPositionAsync(target_position.x_val, target_position.y_val, target_position.z_val, velocity=10).join()

airsim.wait_key('Press any key to reset to original state')
client.reset()
client.armDisarm(False)
client.enableApiControl(False)

