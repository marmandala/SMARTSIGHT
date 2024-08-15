import setup_path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import airsim
import cv2
import numpy as np
import rospy
from scipy.interpolate import interp1d
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
import math
import time

# Initialize AirSim client
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Initialize ROS publishers
pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
pub_point_cloud = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
pub_path = rospy.Publisher('/path_marker', Marker, queue_size=10)

# Initialize ROS node
rospy.init_node('depth_to_point_cloud', anonymous=True)
rate = rospy.Rate(30)  # 30 Hz

# Define interpolation functions based on given points
points = [(0, 0), (0.8, 55), (0.9, 101), (1, 109), (2, 134), (3, 144), (4, 150), (5, 155), (6, 158), (7, 162), (8, 164), (9, 167), (10, 169), (70, 255)]
x_values, y_values = zip(*points)
interpolation_function = interp1d(x_values, y_values, kind='cubic')
inverse_interpolation_function = interp1d(y_values, x_values, kind='cubic')

# Camera parameters
image_width = 256
image_height = 144
fov = 90.0  # Field of view in degrees
cx = image_width / 2
cy = image_height / 2
fx = image_width / (2 * np.tan(np.deg2rad(fov) / 2))
fy = fx

# Space parameters
space_size = 3.5  # meters
num_cells = 20
cell_size = space_size / num_cells

def generate_point_cloud(depth_image):
    start_time = time.time()
    u, v = np.meshgrid(np.arange(image_width), np.arange(image_height))
    z = inverse_interpolation_function(depth_image)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    print(f"generate_point_cloud execution time: {time.time() - start_time} seconds")
    return points

def create_occupied_cells(points):
    start_time = time.time()
    occupied_cells = set()

    for point in points:
        x, y, z = point
        # Shift coordinates to have the drone at the back center of the grid
        x_shifted = x + space_size / 2
        y_shifted = y + space_size / 2
        z_shifted = z

        if 0 <= x_shifted < space_size and 0 <= y_shifted < space_size and 0 <= z_shifted < space_size:
            cell_x = int(x_shifted / cell_size)
            cell_y = int(y_shifted / cell_size)
            cell_z = int(z_shifted / cell_size)
            occupied_cells.add((cell_x, cell_y, cell_z))
    print(f"create_occupied_cells execution time: {time.time() - start_time} seconds")
    return occupied_cells

def publish_occupied_cells(occupied_cells):
    start_time = time.time()
    ground_cells = identify_ground_cells(occupied_cells)

    ground_marker = Marker()
    ground_marker.header.frame_id = "world_enu"
    ground_marker.header.stamp = rospy.Time.now()
    ground_marker.ns = "ground_cells"
    ground_marker.id = 1
    ground_marker.type = Marker.CUBE_LIST
    ground_marker.action = Marker.ADD
    ground_marker.scale.x = cell_size
    ground_marker.scale.y = cell_size
    ground_marker.scale.z = cell_size
    ground_marker.color.r = 0.0
    ground_marker.color.g = 1.0
    ground_marker.color.b = 0.0
    ground_marker.color.a = 1.0

    obstacle_marker = Marker()
    obstacle_marker.header.frame_id = "world_enu"
    obstacle_marker.header.stamp = rospy.Time.now()
    obstacle_marker.ns = "obstacle_cells"
    obstacle_marker.id = 0
    obstacle_marker.type = Marker.CUBE_LIST
    obstacle_marker.action = Marker.ADD
    obstacle_marker.scale.x = cell_size
    obstacle_marker.scale.y = cell_size
    obstacle_marker.scale.z = cell_size
    obstacle_marker.color.r = 0.0
    obstacle_marker.color.g = 1.0
    obstacle_marker.color.b = 0.0
    obstacle_marker.color.a = 1

    for cell in occupied_cells:
        x, y, z = cell
        p = Point()
        p.x = x * cell_size - space_size / 2
        p.y = z * cell_size - cell_size / 2
        p.z = -y * cell_size + space_size / 2

        if cell in ground_cells:
            ground_marker.points.append(p)
        else:
            obstacle_marker.points.append(p)
            

    pub_marker.publish(ground_marker)
    pub_marker.publish(obstacle_marker)
    print(f"publish_occupied_cells execution time: {time.time() - start_time} seconds")



def publish_point_cloud(points):
    start_time = time.time()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world_enu"

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    point_cloud_msg = pc2.create_cloud(header, fields, points)

    # Swap y and z coordinates
    transformed_points = []
    for point in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        transformed_points.append([point[0], point[2], -point[1]])

    point_cloud_msg = pc2.create_cloud_xyz32(header, transformed_points)
    pub_point_cloud.publish(point_cloud_msg)
    print(f"publish_point_cloud execution time: {time.time() - start_time} seconds")




# RRT* algorithm implementation
class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0

def dist(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2 + (node1.z - node2.z)**2)
    

def get_nearest_node(nodes, node):
    nearest_node = nodes[0]
    min_dist = dist(nearest_node, node)
    for n in nodes:
        d = dist(n, node)
        if d < min_dist:
            nearest_node = n
            min_dist = d
    return nearest_node

def is_collision(node1, node2, occupied_cells):
    steps = int(dist(node1, node2) / cell_size)
    if steps == 0:
        steps = 1
    for i in range(steps + 1):
        x = int((node1.x + (node2.x - node1.x) * i / steps) / cell_size)
        y = int((node1.y + (node2.y - node1.y) * i / steps) / cell_size)
        z = int((node1.z + (node2.z - node1.z) * i / steps) / cell_size)
        if (x, y, z) in occupied_cells:
            return True
    return False

def rewire(nodes, new_node, radius, occupied_cells):
    for n in nodes:
        if n != new_node and dist(n, new_node) < radius:
            if not is_collision(n, new_node, occupied_cells):
                cost = new_node.cost + dist(n, new_node)
                if cost < n.cost:
                    n.parent = new_node
                    n.cost = cost

def generate_path(goal_node):
    start_time = time.time()
    path = []
    node = goal_node
    while node.parent is not None:
        path.append(node)
        node = node.parent
    path.append(node)
    print(f"generate_path execution time: {time.time() - start_time} seconds")
    return path[::-1]

def publish_path(path):
    start_time = time.time()
    if not path:
        return
    
    # Publish path as points
    points_marker = Marker()
    points_marker.header.frame_id = "world_enu"
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "path_points"
    points_marker.id = 1
    points_marker.type = Marker.POINTS
    points_marker.action = Marker.ADD
    points_marker.scale.x = 0.1
    points_marker.scale.y = 0.1
    points_marker.color.r = 1.0
    points_marker.color.g = 0.0
    points_marker.color.b = 0.0
    points_marker.color.a = 1.0

    # Publish path as lines
    lines_marker = Marker()
    lines_marker.header.frame_id = "world_enu"
    lines_marker.header.stamp = rospy.Time.now()
    lines_marker.ns = "path_lines"
    lines_marker.id = 2
    lines_marker.type = Marker.LINE_STRIP
    lines_marker.action = Marker.ADD
    lines_marker.scale.x = 0.05
    lines_marker.color.r = 0.0
    lines_marker.color.g = 0.0
    lines_marker.color.b = 1.0
    lines_marker.color.a = 1.0

    first_point = path[1] if len(path) > 1 else path[0]  # Get the first point to move towards
    #print(f"First point coordinates: x={first_point.x - space_size / 2}, y={first_point.z}, z={-first_point.y + space_size / 2}")

    for node in path:
        p = Point()
        p.x = node.x - space_size / 2
        p.y = node.z  # z -> y
        p.z = -node.y + space_size / 2  # -y -> z
        points_marker.points.append(p)
        lines_marker.points.append(p)

    pub_path.publish(points_marker)
    pub_path.publish(lines_marker)
    print(f"publish_path execution time: {time.time() - start_time} seconds")


def rrt_star(start_node, goal_node, occupied_cells, ground_cells, max_iter, radius, opt_iter, index):
    start_time = time.time()
    nodes = [start_node]

    for i in range(max_iter):
        rand_node = get_random_node_above_ground(ground_cells, index)
        nearest_node = get_nearest_node(nodes, rand_node)
        theta = math.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
        phi = math.atan2(rand_node.z - nearest_node.z, math.sqrt((rand_node.x - nearest_node.x)**2 + (rand_node.y - nearest_node.y)**2))
        new_node = Node(nearest_node.x + radius * math.cos(phi) * math.cos(theta),
                        nearest_node.y + radius * math.cos(phi) * math.sin(theta),
                        nearest_node.z + radius * math.sin(phi))

        if not is_collision(nearest_node, new_node, occupied_cells):
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + dist(nearest_node, new_node)
            nodes.append(new_node)
            rewire(nodes, new_node, radius, occupied_cells)
            if dist(new_node, goal_node) < radius:
                goal_node.parent = new_node
                goal_node.cost = new_node.cost + dist(new_node, goal_node)
                nodes.append(goal_node)
                path = generate_path(goal_node)
                publish_rrt_tree(nodes)  # Publish the RRT* tree
                print(f"rrt_star execution time: {time.time() - start_time} seconds")
                return optimize_path(path, occupied_cells, opt_iter)

    publish_rrt_tree(nodes)  # Publish the RRT* tree even if the goal is not reached
    return None


def get_random_node_above_ground(ground_cells, index):
    if index == 1: # left
        max_y = get_max_ground_height(ground_cells)
        x = random.uniform(0, space_size/2)
        z = random.uniform(0, space_size)
        y = (space_size/2) - 0.1
        return Node(x, y, z)
    elif index == 2: # right
        x = random.uniform(space_size/2, space_size)
        z = random.uniform(0, space_size)
        y = (space_size/2) - 0.1
        return Node(x, y, z)
    elif index == 0: # both
        x = random.uniform(0, space_size)
        z = random.uniform(0, space_size)
        y = (space_size/2) - 0.1
        return Node(x, y, z)
    

def get_max_ground_height(ground_cells):
    if not ground_cells:
        return space_size  # Если нет ячеек земли, возвращаем 0
    return space_sizes
    #max_height = max(cell[1] * cell_size for cell in ground_cells)  # Находим максимальное значение y
    #return max_height
    

def optimize_path(path, occupied_cells, opt_iter):
    start_time = time.time()
    def local_planner(node1, node2):
        if not is_collision(node1, node2, occupied_cells):
            print(f"optimize_path execution time: {time.time() - start_time} seconds")
            return dist(node1, node2)
        print(f"optimize_path execution time: {time.time() - start_time} seconds")
        return float('inf')
        
    
    for _ in range(opt_iter):
        path_length = len(path)
        for i in range(path_length - 1):
            for j in range(i + 2, path_length):
                if local_planner(path[i], path[j]) < float('inf'):
                    path = path[:i+1] + path[j:]
                    path_length = len(path)
                    break
    return path

def publish_space_bounds():
    start_time = time.time()
    marker = Marker()
    marker.header.frame_id = "world_enu"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "space_bounds"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.015  # Line width
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    bounds = [
        (0, 0, 0), (space_size, 0, 0),
        (space_size, 0, 0), (space_size, space_size, 0),
        (space_size, space_size, 0), (0, space_size, 0),
        (0, space_size, 0), (0, 0, 0),
        (0, 0, space_size), (space_size, 0, space_size),
        (space_size, 0, space_size), (space_size, space_size, space_size),
        (space_size, space_size, space_size), (0, space_size, space_size),
        (0, space_size, space_size), (0, 0, space_size),
        (0, 0, 0), (0, 0, space_size),
        (space_size, 0, 0), (space_size, 0, space_size),
        (space_size, space_size, 0), (space_size, space_size, space_size),
        (0, space_size, 0), (0, space_size, space_size)
    ]

    for i in range(0, len(bounds), 2):
        p1 = Point(bounds[i][0] - space_size / 2, bounds[i][2], -bounds[i][1] + space_size / 2)  # y -> z
        p2 = Point(bounds[i+1][0] - space_size / 2, bounds[i+1][2], -bounds[i+1][1] + space_size / 2)  # y -> z
        marker.points.append(p1)
        marker.points.append(p2)

    pub_marker.publish(marker)
    print(f"publish_space_bounds execution time: {time.time() - start_time} seconds")

def identify_ground_cells(occupied_cells):
    start_time = time.time()
    
    ground_cells = set()
    ground_threshold = 20  # Определяет сколько клеток по горизонтали должно быть для идентификации как земля

    # Создаем множества для быстрого доступа к занятым клеткам
    occupied_set = set(occupied_cells)

    # Создаем словарь для подсчета количества горизонтальных соседей для каждой ячейки
    horizontal_neighbors = {}
    for x, y, z in occupied_cells:
        if (x, y) not in horizontal_neighbors:
            horizontal_neighbors[(x, y)] = set()
        horizontal_neighbors[(x, y)].add(z)

    for (x, y), z_values in horizontal_neighbors.items():
        for z in z_values:
            horizontal_count = 0
            # Проверяем соседей только по горизонтали в пределах ground_threshold
            for dx in range(-ground_threshold, ground_threshold + 1):
                if (x + dx, y, z) in occupied_set:
                    horizontal_count += 1
                if horizontal_count >= ground_threshold:
                    ground_cells.add((x, y, z))
                    break
            if horizontal_count >= ground_threshold:
                break

    print(f"identify_ground_cells execution time: {time.time() - start_time} seconds")
    return ground_cells


    

buffer_size = 0.6

def create_buffered_cells(occupied_cells, buffer_size):
    start_time = time.time()
    buffer_cells = set()
    buffer_radius = int(buffer_size / cell_size)
    
    for cell in occupied_cells:
        x, y, z = cell
        for dx in range(-buffer_radius, buffer_radius + 1):
            for dy in range(-buffer_radius, buffer_radius + 1):
                for dz in range(-buffer_radius, buffer_radius + 1):
                    buffered_cell = (x + dx, y + dy, z + dz)
                    buffer_cells.add(buffered_cell)
    print(f"create_buffered_cells execution time: {time.time() - start_time} seconds")       
    return buffer_cells
    

def publish_buffered_cells(buffer_cells):
    start_time = time.time()
    buffer_marker = Marker()
    buffer_marker.header.frame_id = "world_enu"
    buffer_marker.header.stamp = rospy.Time.now()
    buffer_marker.ns = "buffer_cells"
    buffer_marker.id = 2
    buffer_marker.type = Marker.CUBE_LIST
    buffer_marker.action = Marker.ADD
    buffer_marker.scale.x = cell_size
    buffer_marker.scale.y = cell_size
    buffer_marker.scale.z = cell_size
    buffer_marker.color.r = 0.5
    buffer_marker.color.g = 0.5
    buffer_marker.color.b = 0.5
    buffer_marker.color.a = 0.25

    for cell in buffer_cells:
        x, y, z = cell
        p = Point()
        p.x = x * cell_size - space_size / 2
        p.y = z * cell_size - cell_size / 2
        p.z = -y * cell_size + space_size / 2
        buffer_marker.points.append(p)

    pub_marker.publish(buffer_marker)
    print(f"publish_buffered_cells execution time: {time.time() - start_time} seconds")

def publish_rrt_tree(nodes):
    start_time = time.time()
    tree_marker = Marker()
    tree_marker.header.frame_id = "world_enu"
    tree_marker.header.stamp = rospy.Time.now()
    tree_marker.ns = "rrt_tree"
    tree_marker.id = 0
    tree_marker.type = Marker.LINE_LIST
    tree_marker.action = Marker.ADD
    tree_marker.scale.x = 0.01
    tree_marker.color.r = 0.0
    tree_marker.color.g = 0.0
    tree_marker.color.b = 1.0
    tree_marker.color.a = 0.6

    for node in nodes:
        if node.parent:
            p1 = Point(node.x - space_size / 2, node.z, -node.y + space_size / 2)
            p2 = Point(node.parent.x - space_size / 2, node.parent.z, -node.parent.y + space_size / 2)
            tree_marker.points.append(p1)
            tree_marker.points.append(p2)

    pub_marker.publish(tree_marker)
    print(f"publish_rrt_tree execution time: {time.time() - start_time} seconds")


def get_first_path_point(index):
    start_time = time.time()
    goal_node = Node(space_size/2, space_size/2, space_size)
    
    rawImage = client.simGetImage("0", airsim.ImageType.DepthVis)
    if rawImage is None:
        print("Camera is not returning image, please check AirSim for error messages")
        return None
    else:
        png = cv2.imdecode(np.frombuffer(rawImage, np.uint8), cv2.IMREAD_UNCHANGED)
        gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)

        point_cloud_points = generate_point_cloud(gray)
        occupied_cells = create_occupied_cells(point_cloud_points)

        buffer_cells = create_buffered_cells(occupied_cells, buffer_size)

        combined_cells = occupied_cells.union(buffer_cells)

        publish_point_cloud(point_cloud_points)
        publish_occupied_cells(occupied_cells)
        #publish_buffered_cells(buffer_cells)
        publish_space_bounds()

        #ground_cells = identify_ground_cells(combined_cells)
        start_node = Node(space_size / 2, space_size / 2, 0)

        path = rrt_star(start_node, goal_node, combined_cells, None, max_iter=4000, radius=1.5, opt_iter=3, index=index)
        if path:
            first_point = path[1] if len(path) > 1 else path[0]
            first_point_coordinates = (first_point.x - space_size / 2, first_point.z, -first_point.y + space_size / 2)
            publish_path(path)
            print(f"get_first_path_point execution time: {time.time() - start_time} seconds     {first_point_coordinates}")
            return first_point_coordinates
        else:
            print(f"Error")
            return None


def main():
    start_time = time.time()
    while not rospy.is_shutdown():
        first_point = get_first_path_point(0)
        if first_point:
            print(f"Next target point: x={first_point[0]}, y={first_point[1]}, z={first_point[2]}")
            print(f"main execution time: {time.time() - start_time} seconds")
        rate.sleep()

if __name__ == '__main__':
    main()
