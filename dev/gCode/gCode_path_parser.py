# Matt's code

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

angle_thresh = 45

# TODO: make this actually parce gcode + nc files...
def read_gcode_csv(file_path):
    df = pd.read_csv(file_path)
    return df


# geo function
def line_from_point_point(pt1, pt2):
    """
    Takes in two points of the form, [x,y]
    Returns slope, "m", and y-intercept, "b"
    """
    m = (pt1[1] - pt2[1]) / (pt1[0] - pt2[0])
    b = pt2[1] - (m * pt2[0])
    return [m, b]


# geo function
def calculate_distance(p1, p2):
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


# TODO: this needs to be fixed to properly create paths
def gen_point_to_point_angles(gcode_df, gantry_orientation):
    # compute direction between point i and point i+1
    # see if it is within 45 deg of gantry orientation
    # label green or red, continue
    gcode_df["can_cut"] = False
    gcode_df["angle_d"] = 0
    for i in range(len(gcode_df) - 1):
        print("pt1 = " + str([gcode_df.x[i], gcode_df.y[i]]))
        print("pt2 = " + str([gcode_df.x[i + 1], gcode_df.y[i + 1]]))
        mi, bi = line_from_point_point(
            [gcode_df.x[i], gcode_df.y[i]], [gcode_df.x[i + 1], gcode_df.y[i + 1]]
        )
        if np.isnan(mi):
            segment_angle = 0
        else:
            segment_angle = 90 - np.rad2deg(np.arctan(mi))
        # storing point to point angle here in deg
        gcode_df.loc[i, "angle_d"] = segment_angle

        # TODO: need to make sure we stay on one side of this angle for a continuous segment
        # ie we cannot go from 30 to -30 or somebting even though it's in range.....
        # if abs(segment_angle) - gantry_orientation < 45:
        # gcode_df.loc[i, "can_cut"] = True

    return gcode_df


# This seems to be working, getting
def create_paths_by_angle(gcode_df):
    initial_angle = gcode_df.iloc[0].angle_d
    paths = []
    current_path = []
    min_angle_d = initial_angle
    max_angle_d = initial_angle
    path_distance_mm = 0

    for i in range(len(gcode_df)):
        current_angle = gcode_df.iloc[i].angle_d
        point = [gcode_df.iloc[i].x, gcode_df.iloc[i].y, gcode_df.iloc[i].z]

        if (
            abs(current_angle - initial_angle) <= 45
        ):  # compare with initial angle of seggy
            if current_path:
                path_distance_mm += calculate_distance(current_path[-1], point)
            current_path.append(point)
            min_angle_d = min(min_angle_d, current_angle)
            max_angle_d = max(max_angle_d, current_angle)
        else:  # angle too large, close current path and start a new one
            if current_path:
                # add current_path to paths list and reset current_path
                paths.append(
                    {
                        "path_points": current_path,
                        "min_angle_d": min_angle_d,
                        "max_angle_d": max_angle_d,
                        "num_points": len(current_path),
                        "path_distance_mm": path_distance_mm,
                    }
                )
                current_path = []
                path_distance_mm = 0
                initial_angle = current_angle  # start new seggy
                min_angle_d = current_angle
                max_angle_d = current_angle
                current_path.append(point)

    # add last path if it's not empty
    if current_path:
        paths.append(
            {
                "path_points": current_path,
                "min_angle_d": min_angle_d,
                "max_angle_d": max_angle_d,
                "num_points": len(current_path),
                "path_distance_mm": path_distance_mm,
            }
        )

    paths_df = pd.DataFrame(
        paths,
        columns=[
            "path_points",
            "min_angle_d",
            "max_angle_d",
            "num_points",
            "path_distance_mm",
        ],
    )

    return paths_df


def plot_paths(gcode_df, filtered_paths):
    plt.figure(figsize=(10, 6))

    for path in filtered_paths:
        path_points = gcode_df.iloc[path]
        x = path_points["x"]
        y = path_points["y"]

        plt.scatter(x, y, label="Path Points")

        # Optionally plot lines connecting points in the path
        plt.plot(x, y, linestyle="-", marker="o", label="Path")

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Filtered Paths")
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_path_by_angle(paths_df):
    plt.figure(figsize=(10, 6))

    for i, path in paths_df.iterrows():
        x = [point[0] for point in path["path_points"]]
        y = [point[1] for point in path["path_points"]]
        plt.plot(x, y, label=f"Path {i+1}")

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Plot of Paths by Angle")
    plt.grid(True)
    plt.legend()
    plt.show()


# all we need to add:
# compute necessary gantry orientations given our paths and path angle min/max
# optimize for minimizing # of segmennts and also maximuizing path length
# also minimize unique gantry orientations


def main():
    my_df = read_gcode_csv("dev/gCode/basePlate_test.csv")
    plt.scatter(my_df.x, my_df.y)
    plt.grid()
    plt.title("Plotting whole GCODE Path (just the points)")
    plt.show()

    gcode_df_with_path_flag = gen_point_to_point_angles(my_df, 0)
    # i might be creating paths from all of it and not just what i can cut from initial path flag
    paths_list_angle_filtered = create_paths_by_angle(gcode_df_with_path_flag)
    plot_path_by_angle(paths_list_angle_filtered)


if __name__ == "__main__":
    main()
