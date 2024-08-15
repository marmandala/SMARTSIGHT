import pygame
import math
from queue import PriorityQueue
import random

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
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
        self.color = ORANGE

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
        self.color = TURQUOISE

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


def randomize_barriers(grid, barrier_radius):
    for row in grid:
        for spot in row:
            if random.random() < 0.01:
                if not any(s.is_barrier for s in spot.neighbors):
                    spot.make_barrier()


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


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


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
    meters_per_pixel = 5 / 800  # Количество метров в одном пикселе

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
        start = simplified_coords_from_start[0]
        current = simplified_coords_from_start[i]
        if i < len(simplified_coords_from_start) - 1:
            next_point = simplified_coords_from_start[i + 1]
            x_meters = ((current[0] - next_point[0]) * meters_per_pixel * -1)
            y_meters = ((current[1] - next_point[1]) * meters_per_pixel)
            print(i, x_meters, y_meters)
            i = i + 1

    print("Количество точек для построения пути:", len(simplified_coords), "   simplified_coords = ", simplified_coords)
    return simplified_coords


def draw_path(draw):
    global simplified_coords
    #print("Количество точек для построения пути:", len(path_coords), "   path_coords = ", path_coords)
    for i in range(len(simplified_coords) - 1):
        pygame.draw.line(WIN, PURPLE, simplified_coords[i], simplified_coords[i + 1], 5)
    # Отрисовка черных точек на месте финальных точек маршрута
    for point in simplified_coords:
        pygame.draw.circle(WIN, BLACK, point, 3)
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


def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid


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
    draw_buffer(win, grid, buffer_width)
    draw_path(lambda: draw(win, grid, ROWS, width, buffer_width))
    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col


def main(win, width):
    ROWS = 35
    buffer_width = 2
    circle_diameter = 5
    grid = make_grid(ROWS, width)

    start = None
    end = None

    run = True
    while run:
        draw(win, grid, ROWS, width, buffer_width)

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

            # Средняя кнопка мыши
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 2:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                add_circle_obstacle(row, col, circle_diameter, grid)

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
        if start != None and end != None:
            for row in grid:
                for spot in row:
                    spot.update_neighbors(grid, allow_diagonals=True)
            algorithm(lambda: draw(win, grid, ROWS, width, buffer_width), grid, start, end)

    pygame.quit()


main(WIN, WIDTH)
