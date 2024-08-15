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

# Заданные точки (x, y) для интерполяции
points = [(0, 0), (0.8, 55), (0.9, 101), (1, 109), (2, 134), (3, 144), (4, 150), (5, 155), (6, 158), (7, 162), (8, 164), (9, 167), (10, 169), (70, 255)]

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
    rawImage = client.simGetImage("0", airsim.ImageType.DepthVis)
    
    if rawImage is None:
        print("Camera is not returning image, please check airsim for error messages")
        airsim.wait_key("Press any key to exit")
        sys.exit(0)
    else:
        png = cv2.imdecode(np.frombuffer(rawImage, np.uint8), cv2.IMREAD_UNCHANGED)
        gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)

        # Display depth image using OpenCV
        displayDepthImage(gray)

        # Get the center pixel value
        center_pixel_value = gray[gray.shape[0] // 2, gray.shape[1] // 2]

        # Calculate the distance to the nearest object in front of the camera using inverse interpolation
        distance = inverse_interpolation_function(center_pixel_value)

        # Output the distance estimate to the console
        print("Distance =", distance, center_pixel_value)
