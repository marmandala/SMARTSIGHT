import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def add_cube(x, y, z):
    rospy.init_node('add_cube_node', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(1)  # Ждем инициализации узла

    marker = Marker()
    marker.header.frame_id = "world_enu"  # Меняйте на нужный frame_id вашей сцены в RViz
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 0.7  # Прозрачность
    marker.color.r = 1.0  # Красный
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z

    marker_pub.publish(marker)


if __name__ == '__main__':
    try:
        while True:
            x = float(input("Введите координату x: "))
            y = float(input("Введите координату y: "))
            z = float(input("Введите координату z: "))
            add_cube(x, y, z)
            print("Куб добавлен")
    except rospy.ROSInterruptException:
        pass
