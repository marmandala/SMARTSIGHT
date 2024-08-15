import setup_path
import airsim
import cv2
import numpy as np
import sys
from scipy.interpolate import interp1d
import math
from queue import PriorityQueue
import pygame

buffer_width = 1
ROWS = 35
WIDTH = 600
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

# Размер сетки в метрах
grid_size_meters = 5
# Размер сетки в пикселях (35x35 клеток)
grid_size_pixels = 35

RED = (240, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (10, 10, 10)
PURPLE = (128, 0, 128)
ORANGE = (250, 131, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.is_barrier = False
        self.is_buffer = False
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def reset(self):
        self.color = WHITE
        self.is_barrier = False
        self.is_buffer = False

    def make_start(self):
        self.color = RED

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.is_barrier = True
        self.color = BLACK

    def make_buffer(self):
        self.is_buffer = True
        self.color = GREY

    def make_end(self):
        self.color = GREEN

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid, allow_diagonals=False):
        self.neighbors = []
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # Смежные ячейки по горизонтали и вертикали

        if allow_diagonals:
            directions += [(1, 1), (-1, -1), (1, -1), (-1, 1)]  # Диагональные соседи

        for dr, dc in directions:
            new_row, new_col = self.row + dr, self.col + dc
            if 0 <= new_row < self.total_rows and 0 <= new_col < self.total_rows:
                neighbor = grid[new_row][new_col]
                if not neighbor.is_buffer:
                    self.neighbors.append(neighbor)

    def __lt__(self, other):
        return False

def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid


def draw_buffer(win, grid, buffer_width):
    for row in grid:
        for spot in row:
            if spot.is_barrier:
                for i in range(-buffer_width, buffer_width + 1):
                    for j in range(-buffer_width, buffer_width + 1):
                        new_row, new_col = spot.row + i, spot.col + j
                        if 0 <= new_row < spot.total_rows and 0 <= new_col < spot.total_rows:
                            if not grid[new_row][new_col].is_barrier and not grid[new_row][new_col].is_buffer:
                                grid[new_row][new_col].make_buffer()


def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width, buffer_width):
    win.fill(WHITE)
    for row in grid:
        for spot in row:
            spot.draw(win)
    draw_grid(win, rows, width)
    draw_path(lambda: draw(win, grid, ROWS, width, buffer_width))
    pygame.display.update()


def displayDepthImage(depth_image):
    cv2.imshow("Depth Image", depth_image)
    cv2.waitKey(1)


def calculate_cylinder_width(distance, image_width, image_resolution, obstacle_width):
    focal_length = 1.1
    sensor_width = 3.68
    pixel_size = sensor_width / image_resolution
    alpha = 2 * math.atan((image_width / 2) * pixel_size / focal_length)
    width = 2 * distance * math.tan(0.5 * alpha)
    return width


client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()
pose = client.simGetVehiclePose()
movement_vector = airsim.Vector3r(1, 0, 0)
target_position = pose.position + movement_vector
client.moveToPositionAsync(target_position.x_val, target_position.y_val, target_position.z_val, velocity=3).join()

# Заданные точки (x, y) для интерполяции расстояний
points = [(0, 0), (0.2, 84), (0.3, 122), (0.4, 132), (0.5, 138), (0.6, 142), (0.7, 146), (0.8, 149), (0.9, 151), (1, 154),
          (2, 168), (3, 177), (4, 183), (5, 188), (6, 192), (7, 196), (8, 198), (9, 201), (10, 204), (70, 255)]
x_values, y_values = zip(*points)
inverse_interpolation_function = interp1d(y_values, x_values, kind='cubic')


def add_obstacles_to_grid(grid, obstacles, grid_size_pixels):
    for row in grid:
        for spot in row:
            if spot.is_barrier or spot.is_buffer:
                spot.reset()
    for obstacle in obstacles:
        distance_cells, offset_x_cells, width_cells = obstacle
        # Переводим координаты препятствия на сетке в индексы массива grid
        obstacle_col = ROWS - 1 - distance_cells
        obstacle_row = ROWS//2 + offset_x_cells
        # Учитываем ширину препятствия при установке барьеров на сетку
        for row in range(obstacle_row - width_cells // 2, obstacle_row + width_cells // 2 + 1):
            for col in range(obstacle_col - width_cells // 2, obstacle_col + width_cells // 2 + 1):
                if 0 <= row < ROWS and 0 <= col < ROWS:
                    grid[row][col].make_barrier()
                    draw_buffer(WIN, grid, buffer_width)
                    

# Создаем список для хранения координат пути
path_coords = []
fin_coords = []
simplified_coords = []

def reconstruct_path(came_from, current, draw):
    global path_coords
    path_coords = []
    prev_point = None
    while current in came_from:
        current = came_from[current]
        path_coords.append((current.x + current.width // 2, current.y + current.width // 2))
    draw_path(draw)
    simple_coords(path_coords)

def simple_coords(raw_coords):
    global simplified_coords
    simplified_coords = []
    meters_per_pixel = 5 / 600  # Количество метров в одном пикселе

    if len(raw_coords) < 2:
        return raw_coords  # Нет необходимости упрощать, если меньше двух точек
    
    prev_dx = raw_coords[1][0] - raw_coords[0][0]
    prev_dy = raw_coords[1][1] - raw_coords[0][1]
    
    simplified_coords.append(raw_coords[0])
    
    for i in range(1, len(raw_coords) - 1):
        current = raw_coords[i]
        next_point = raw_coords[i + 1]
        dx = next_point[0] - current[0]
        dy = next_point[1] - current[1]
        # Проверяем, если направление изменилось от предыдущей точки
        if dx != prev_dx or dy != prev_dy:
            simplified_coords.append(current)
            prev_dx = dx
            prev_dy = dy
    
    # В конце добавляем последнюю точку
    simplified_coords.append(raw_coords[-1])
    
    print("Координаты точек пути в метрах:")
    i = 0
    simplified_coords_from_start = simplified_coords[::-1]
    for point in simplified_coords_from_start:
        current = simplified_coords_from_start[i]
        if i < len(simplified_coords_from_start) - 1: 
            next_point = simplified_coords_from_start[i+1]
            x_meters = ((current[0] - next_point[0]) * meters_per_pixel*-1)
            y_meters = ((current[1] - next_point[1]) * meters_per_pixel)
            pose = client.simGetVehiclePose()
            movement_vector = airsim.Vector3r(y_meters*10, x_meters*10, 0)
            target_position = pose.position + movement_vector
            client.moveToPositionAsync(target_position.x_val, target_position.y_val, target_position.z_val, velocity=3).join()
            print(i, x_meters, y_meters)
            i = i + 1
    client.reset()
    client.armDisarm(False)
    client.enableApiControl(False)


def draw_path(draw):
    global simplified_coords
    #print("Количество точек для построения пути:", len(path_coords), "   path_coords = ", path_coords)
    for i in range(len(simplified_coords) - 1):
        pygame.draw.line(WIN, ORANGE, simplified_coords[i], simplified_coords[i + 1], 5)
    # Отрисовка черных точек на месте финальных точек маршрута
    for point in simplified_coords:
        pygame.draw.circle(WIN, PURPLE, point, 3)
    pygame.display.update()

def algorithm(draw, grid, start, end):
    for row in grid:
        for spot in row:
            if not (spot.is_barrier or spot.is_buffer or spot.color == TURQUOISE or spot.color == ORANGE):
                spot.reset()
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    open_set_hash = {start}

    while not open_set.empty():
        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                if not neighbor.is_barrier or neighbor.is_buffer:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)

    return False


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col

def main(win, width):
    start = None
    end = None
    grid = make_grid(ROWS, width)
    
    while True:
        obstacles = []  # Обновляем список препятствий на каждой итерации
        rawImage = client.simGetImage("0", airsim.ImageType.DepthVis)
        
        if rawImage is None:
            print("Camera is not returning image, please check airsim for error messages")
            airsim.wait_key("Press any key to exit")
            sys.exit(0)
        else:
            png = cv2.imdecode(np.frombuffer(rawImage, np.uint8), cv2.IMREAD_UNCHANGED)
            
            # Определите верхнюю границу области интереса (ROI)
            roi_top = 0
            roi_bottom = png.shape[0] // 2  # Выбираем верхнюю половину изображения
            
            # Выбираем только верхнюю часть изображения глубины
            roi = png[roi_top:roi_bottom, :]
            
            gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            # Применяем пороговую обработку
            ret, thresh = cv2.threshold(gray_image, 254, 255, cv2.THRESH_BINARY)

            # Применяем морфологическое открытие
            kernel = np.ones((3, 3), np.uint8)
            img_open = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

            # Применяем инверсию порогового значения
            ret1, thresh1 = cv2.threshold(img_open, 254, 255, cv2.THRESH_BINARY_INV)

            # Находим контуры
            contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            img_with_contours = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)  # Для визуализации

            for i, contour in enumerate(contours):
                if cv2.contourArea(contour) > 10:  # Игнорируем маленькие контуры
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x, center_y = x + w // 2, y + h // 2
                    pixel_value = gray_image[center_y, center_x]
                    distance = inverse_interpolation_function(pixel_value)

                    image_center_x = gray_image.shape[1] / 2
                    offset_x_pixels = center_x - image_center_x
                    # Предполагаем, что угловое поле зрения камеры составляет 90 градусов
                    offset_x_meters = (offset_x_pixels / gray_image.shape[1]) * 2 * distance * np.tan(np.radians(60))

                    # Вычисляем ширину препятствия
                    obstacle_width = calculate_cylinder_width(distance, w, gray_image.shape[1], w)

                    # Переводим расстояние, смещение и ширину в клетки сетки
                    distance_cells = int(distance / grid_size_meters * grid_size_pixels)
                    offset_x_cells = int(offset_x_meters / grid_size_meters * grid_size_pixels)
                    width_cells = int(obstacle_width / grid_size_meters * grid_size_pixels)

                    # Добавляем данные о препятствии в список
                    obstacles.append((distance_cells, offset_x_cells, width_cells))

                    # Выводим расстояние, смещение и ширину в клетках
                    #print(f"Obstacle {i}: Distance = {distance}({distance_cells}), Offset X = {offset_x_meters}({offset_x_cells}), Width = {obstacle_width}({width_cells})")

                    cv2.rectangle(img_with_contours, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(img_with_contours, f"{distance}({distance_cells}), offsetX={offset_x_meters}({offset_x_cells}), Width={obstacle_width}({width_cells})", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            displayDepthImage(img_with_contours)

        # Добавляем препятствия на сетку после получения данных изображения
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                
            if pygame.mouse.get_pressed()[0]:  # Левая кнопка мыши
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()
                elif not end and spot != start:
                    end = spot
                    end.make_end()
                elif spot != end and spot != start:
                    barrier_radius = 2
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # Правая кнопка мыши
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None
                
            if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE and start and end:
                        for row in grid:
                            for spot in row:
                                spot.update_neighbors(grid, allow_diagonals=True)
                        algorithm(lambda: draw(win, grid, ROWS, width, buffer_width), grid, start, end)

                    if event.key == pygame.K_c:
                        start = None
                        end = None
                        grid = make_grid(ROWS, width)
        add_obstacles_to_grid(grid, obstacles, grid_size_pixels)
        if start != None and end != None:
            for row in grid:
                for spot in row:
                    spot.update_neighbors(grid, allow_diagonals=True)
            algorithm(lambda: draw(win, grid, ROWS, width, buffer_width), grid, start, end)
        draw(win, grid, ROWS, width, buffer_width)

main(WIN, WIDTH)
