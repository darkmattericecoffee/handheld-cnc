import numpy as np
import matplotlib.pyplot as plt

def sin_generator(num_points, path_max_y, sin_amp, sin_period):
    path_array_x = np.zeros(num_points)
    path_array_y = np.zeros(num_points)

    for i in range(num_points):
        y = path_max_y * i / (num_points - 1)
        x = sin_amp * np.sin((2 * np.pi / sin_period) * y)
        path_array_x[i] = x
        path_array_y[i] = y

    return path_array_x, path_array_y

num_points = 1000
path_max_y = 100
sin_amp = 50
sin_period = 100

path_array_x, path_array_y = sin_generator(num_points, path_max_y, sin_amp, sin_period)

plt.plot(path_array_y, path_array_x)
plt.xlabel("Y-axis")
plt.ylabel("X-axis")
plt.title("Sine Wave Path")
plt.grid()
plt.show()