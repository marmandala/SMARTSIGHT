import airsim
import time
import math
import cv2
import numpy as np


def rotate_image(image, angle):
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated


client = airsim.MultirotorClient()
client.confirmConnection()

while True:
    imu_data = client.getImuData()
    orientation = imu_data.orientation
    pitch, roll, yaw = airsim.to_eularian_angles(orientation)

    print("Roll: {:.2f} degrees".format(math.degrees(roll)))

    rawImage = client.simGetImage("0", airsim.ImageType.Scene)
    if rawImage is None:
        print("Camera is not returning image, please check AirSim for error messages")
    else:
        png = cv2.imdecode(np.frombuffer(rawImage, np.uint8), cv2.IMREAD_UNCHANGED)
        gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)

    angle = -math.degrees(roll)
    stabilized_img = rotate_image(png, angle)

    cv2.imshow("Stabilized Image", stabilized_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

client.armDisarm(False)
client.reset()
client.enableApiControl(False)
cv2.destroyAllWindows()
