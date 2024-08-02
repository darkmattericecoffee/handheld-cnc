import numpy as np
import pandas as pd


def read_gcode_csv(file_path):
    df = pd.read_csv(file_path)
    return df


def line_from_point_point(pt1, pt2):
    """
    Takes in a point of the form, [x,y]
    Returns slope, "m", and y-intercept, "b"
    """
    m = (pt1[1] - pt2[1]) / (pt1[0] - pt2[0])
    b = pt2[1] - (m * pt2[0])
    return [m, b]


def create_segments(gcode_df, gantry_orientation):
    # compute direction between point i and point i+1
    # see if it is within 45 deg of gantry orientation
    # label green or red, continue
    gcode_df["can_cut"] = False
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
        # TODO: need to make sure we stay on one side of this angle for a continuous segment
        # ie we cannot go from 30 to -30 or somebting even though it's in range.....
        if abs(segment_angle) - gantry_orientation < 45:
            gcode_df.loc[i, "can_cut"] = True

    return gcode_df


# take in file
# parse to get x y z
# go through array and see how many points in a row have a diff parallel (or parallel +/-45 deg) to gantry access
# if segment is longer


def main():
    my_df = read_gcode_csv("dev/gCode/basePlate_test.csv")
    gcode_df_with_segment_flag = create_segments(my_df, 0)
    print("hi")


if __name__ == "__main__":
    main()
