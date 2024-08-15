import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

# Заданные точки (x, y)
points = [(0, 0), (0.2, 84), (0.3, 122), (0.4, 132), (0.5, 138), (0.6, 142), (0.7, 146), (0.8, 149), (0.9, 151), (1, 154), (2, 168), (3, 177), (4, 183), (5, 188), (6, 192), (7, 196), (8, 198), (9, 201), (10, 204)]

# Разделение точек на отдельные списки x и y
x_values, y_values = zip(*points)

# Интерполяция для получения функции, проходящей через точки
interpolation_function = interp1d(x_values, y_values, kind='cubic')

# Создание более плотного набора точек для гладкого графика
x_smooth = np.linspace(0, 10, 100)
y_smooth = interpolation_function(x_smooth)

# Построение графика
plt.plot(x_values, y_values, marker='o', linestyle='', color='b', label='Данные точки')
plt.plot(x_smooth, y_smooth, linestyle='-', color='r', label='Интерполяция')

# Настройки графика
plt.title('Интерполяция точек')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()

# Отображение графика
plt.show()
