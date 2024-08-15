import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.reset()
client.armDisarm(False)
client.enableApiControl(False)
