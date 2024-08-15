import point_cloud
import rospy
import threading
import airsim
import matplotlib.pyplot as plt
import time
from queue import Queue

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

airsim.wait_key('Press any key to move vehicle')
print("Starting...")

# Initialize the list to store coordinates
trajectory = [[0, 0, 0]]
index = 0

# Create a queue to communicate between threads
index_queue = Queue()

time_sleep = 0.35

def index_thread():
    global index
    while not rospy.is_shutdown():
        x = index_queue.get()
        if x > 0.8:  # right
            index = 2
            print("RIGHT")
            time.sleep(time_sleep)
            index = 0
        elif x < -0.8:  # left
            index = 1
            print("LEFT")
            time.sleep(time_sleep)
            index = 0
        else:
            print("BOTH")
            index = 0

# Start the index thread
thread1 = threading.Thread(target=index_thread)
thread1.start()

while not rospy.is_shutdown():
    # Получение текущего положения и ориентации квадрокоптера
    pose = client.simGetVehiclePose()
    positions = point_cloud.get_first_path_point(index)

    if positions is not None:
        movement_vector = airsim.Vector3r(positions[1]*2, positions[0]*2, 0)
        target_position = pose.position + movement_vector
        
        # Send target_position.x_val to the index thread via queue
        print(f"X, Y, Z:    {movement_vector.x_val/2}, {movement_vector.y_val/2}, {movement_vector.z_val/2}")
        index_queue.put(movement_vector.y_val/2)
        client.moveToPositionAsync(target_position.x_val, target_position.y_val, -5, velocity=3.8)
        
        
        print(positions[0], positions[1], positions[2])

        # Add the new position incrementally
        last_position = trajectory[-1]
        new_position = [last_position[0] + positions[0], last_position[1] + positions[1], last_position[2] + positions[2]]
        trajectory.append(new_position)
        print(f"Height:{pose.position.z_val}")
    else:
        print("No valid path found, trying again...")

# Signal the index thread to terminate
index_queue.put(None)
thread1.join()

# Visualize the trajectory using matplotlib
trajectory = list(zip(*trajectory))  # Unzip to get x, y, z separately

# Создание графика
plt.figure()
plt.plot(trajectory[0], trajectory[1], label='Path')

# Определение максимального расстояния по обеим осям
max_range = max(max(trajectory[0]) - min(trajectory[0]), max(trajectory[1]) - min(trajectory[1]))

# Установка одинакового диапазона для обеих осей
mid_x = (max(trajectory[0]) + min(trajectory[0])) / 2
mid_y = (max(trajectory[1]) + min(trajectory[1])) / 2

plt.xlim(mid_x - max_range / 2, mid_x + max_range / 2)
plt.ylim(mid_y - max_range / 2, mid_y + max_range / 2)

# Установка равных пропорций осей
plt.gca().set_aspect('equal')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Drone Path Visualization')
plt.legend()
plt.grid(True)
plt.show()

