import setup_path
import airsim
import cv2
import time
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# Connecting to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Adjust these values as needed
forward_velocity = 5
lateral_velocity = 5

a = 0
i = 0

# Move the drone forward
client.moveByVelocityAsync(forward_velocity, 0, 0, 1000)

# Заданные точки (x, y) для интерполяции
points = [(0, 0), (0.2, 84), (0.3, 122), (0.4, 132), (0.5, 138), (0.6, 142), (0.7, 146), (0.8, 149), (0.9, 151), (1, 154), (2, 168), (3, 177), (4, 183), (5, 188), (6, 192), (7, 196), (8, 198), (9, 201), (10, 204), (70, 255)]

# Разделение точек на отдельные списки x и y
x_values, y_values = zip(*points)

# Интерполяция для получения функции, проходящей через точки
interpolation_function = interp1d(x_values, y_values, kind='cubic')

# Инвертирование интерполяции для получения обратной функции
inverse_interpolation_function = interp1d(y_values, x_values, kind='cubic')

def displayDepthImage(depth_image):
    cv2.imshow("Depth Image", depth_image)
    cv2.waitKey(50)

while True:
    
    i = i + 1

    # Capture depth image
    rawImage = client.simGetImage("0", airsim.ImageType.DepthVis)
    
    if rawImage is None:
        print("Camera is not returning image, please check airsim for error messages")
        airsim.wait_key("Press any key to exit")
        sys.exit(0)
    elif i > 1:
        png = cv2.imdecode(np.frombuffer(rawImage, np.uint8), cv2.IMREAD_UNCHANGED)
        gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)

        # Display depth image using OpenCV
        displayDepthImage(gray)

        # Get the center pixel value
        center_pixel_value = gray[gray.shape[0] // 2, gray.shape[1] // 2]

        # Calculate the distance to the nearest object in front of the camera using inverse interpolation
        distance = inverse_interpolation_function(center_pixel_value)

        # Check if the distance is below a certain threshold
        if distance < 1.5:
            # Stop the drone
            print("Whoops - we are about to crash, so stopping!")
            client.hoverAsync().join()
            client.moveByVelocityAsync(0, lateral_velocity, 0, 1).join()
            a = a + 1
            print(a)
            if a > 5:     
                client.reset()
                airsim.wait_key("Press any key to exit")
                sys.exit(0)
        	
        elif distance > 1.5:
            client.moveByVelocityAsync(forward_velocity, 0, 0, 1000)

        # Output the distance estimate to the console
        print("Distance =", distance)

