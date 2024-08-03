from time import sleep
import datetime
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection
from matplotlib.widgets import Slider

TARGET_PATH_X = 0
TARGET_PATH_Y = 1

ACTUAL_PATH_X = 0
ACTUAL_PATH_Y = 1
ACTUAL_PATH_YAW = 2
ACTUAL_PATH_GOAL_X = 3
ACTUAL_PATH_GOAL_Y = 4
ACTUAL_PATH_TOOL_X = 5
ACTUAL_PATH_TOOL_Y = 6
DESIRED_PATH_TOOL_X = 7
DESIRED_PATH_TOOL_Y = 8
ACTUAL_PATH_CUTTING = 9
MOTOR_POS = 10

GANTRY_LENGTH_MM = 106.0

PLOT_PADDING = GANTRY_LENGTH_MM/2 + 10.0

with open('../../../logFiles/logFile_45.txt') as f:
    lines = f.readlines()
    
    target_paths = np.array(
        [x.strip()[5:].split(',')[1:] for x in lines if x.startswith("PATH:")], 
        dtype=float
    )
    target_paths = target_paths[:,:2]
    
    target_paths = target_paths.reshape((-1,1000,2))
    
    actual_path = np.array(
        [x.strip()[4:].split(',') for x in lines if x.startswith("POS:")],
        dtype=float
    ).round(decimals=4)

    actual_path = actual_path[np.insert(np.any(np.diff(actual_path, axis=0) != 0, axis=1), 0, True)]

matplotlib.rcParams['figure.figsize'] = [16, 16]

cutting_status = actual_path[:, ACTUAL_PATH_CUTTING].astype(int) == 1

active_path = actual_path[cutting_status == 1]
inactive_path = actual_path[cutting_status == 0]

actual_x = actual_path[:, ACTUAL_PATH_X]
actual_y = actual_path[:, ACTUAL_PATH_Y]

colors = ['k' if status == 1 else 'gray' for status in cutting_status]
colors = colors[1:]
points = np.array([actual_x, actual_y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
lc = LineCollection(segments, colors=colors)
lc.set_label('Actual Path')

yaw = actual_path[:, ACTUAL_PATH_YAW]
yaw_active = active_path[:, ACTUAL_PATH_YAW]
yaw_inactive = inactive_path[:, ACTUAL_PATH_YAW]

left_x = actual_x - GANTRY_LENGTH_MM/2 * np.cos(yaw)
left_y = actual_y - GANTRY_LENGTH_MM/2 * np.sin(yaw)
right_x = actual_x + GANTRY_LENGTH_MM/2 * np.cos(yaw)
right_y = actual_y + GANTRY_LENGTH_MM/2 * np.sin(yaw)

goal_x = actual_path[:, ACTUAL_PATH_GOAL_X]
goal_y = actual_path[:, ACTUAL_PATH_GOAL_Y]

tool_x = active_path[:, ACTUAL_PATH_TOOL_X]
tool_y = active_path[:, ACTUAL_PATH_TOOL_Y]
tool_x_inactive = inactive_path[:, ACTUAL_PATH_TOOL_X]
tool_y_inactive = inactive_path[:, ACTUAL_PATH_TOOL_Y]

target_x = actual_path[:, DESIRED_PATH_TOOL_X]
target_y = actual_path[:, DESIRED_PATH_TOOL_Y]

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.25)

for i in range(len(target_paths)): 
    target_x = target_paths[i, :, TARGET_PATH_X]
    target_y = target_paths[i, :, TARGET_PATH_Y]
    if i == 0:
        ax.plot(target_x, target_y, label='Target Path', color='b', alpha=0.5)

ax.add_collection(lc)
ax.plot(left_x, left_y, label='Gantry Boundary', color='r', linestyle='dashed')
ax.plot(right_x, right_y, color='r', linestyle='dashed')
tool_scatter = ax.scatter(tool_x, tool_y, label='Current Tool Position', color='orange', marker='.', alpha=0.5)
tool_inactive_scatter = ax.scatter(tool_x_inactive, tool_y_inactive, label='Current Tool Position (inactive)', color='gray', marker='.', alpha=0.5)
target_scatter = ax.scatter(target_x, target_y, label='Target Tool Position', color='red', marker='.', alpha=0.3)
tool_offset_scatter = ax.scatter([], [], label='Current Tool Position (offset)', color='purple', marker='.', alpha=0.5)
tool_offset_inactive_scatter = ax.scatter([], [], label='Current Tool Position (offset inactive)', color='pink', marker='.', alpha=0.5)
ax.set_aspect('equal')
ax.legend()

axcolor = 'lightgoldenrodyellow'
ax_x_offset = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_y_offset = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)

s_x_offset = Slider(ax_x_offset, 'X Offset', -10.0, 10.0, valinit=-3)
s_y_offset = Slider(ax_y_offset, 'Y Offset', -10.0, 10.0, valinit=-3.5)

def update(val):
    x_starting_offset = 0
    y_starting_offset = 0
    x_offset = x_starting_offset + s_x_offset.val
    y_offset = y_starting_offset + s_y_offset.val
    tool_x_offset = active_path[:, ACTUAL_PATH_TOOL_X] + x_offset * np.cos(yaw_active) - y_offset * np.sin(yaw_active)
    tool_y_offset = active_path[:, ACTUAL_PATH_TOOL_Y] + x_offset * np.sin(yaw_active) + y_offset * np.cos(yaw_active)
    tool_x_offset_inactive = inactive_path[:, ACTUAL_PATH_TOOL_X] + x_offset * np.cos(yaw_inactive) - y_offset * np.sin(yaw_inactive)
    tool_y_offset_inactive = inactive_path[:, ACTUAL_PATH_TOOL_Y] + x_offset * np.sin(yaw_inactive) + y_offset * np.cos(yaw_inactive)
    
    tool_offset_scatter.set_offsets(np.c_[tool_x_offset, tool_y_offset])
    tool_offset_inactive_scatter.set_offsets(np.c_[tool_x_offset_inactive, tool_y_offset_inactive])
    
    plt.draw()

s_x_offset.on_changed(update)
s_y_offset.on_changed(update)

plt.show()
