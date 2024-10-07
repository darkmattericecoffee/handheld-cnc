# Cam's code

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

angle_thresh = 45

# TODO: make this actually parce gcode + nc files...
def read_gcode_csv(file_path):
    df = pd.read_csv(file_path)
    new_row = pd.DataFrame([[0, 0, 0]], columns=df.columns)     # add home point to start
    df = pd.concat([new_row, df], ignore_index=True)
    return df

def read_gcode(file_path):
    # dataframe with columns: x, y, z

    return

# geo function
def calculate_distance(p1, p2):
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

# geo function
def get_line_info(p1, p2):
    # takes in two points and calculates the angle and length of the line they make
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    m = (dy) / (dx)
    b = p2[1] - (m * p2[0])

    #dist = (dx**2 + dy**2)**(1/2)
    dist = calculate_distance(p1,p2)

    if np.isnan(m):
        # if dx = dy = 0
        beta = 0
    else:
        # alpha = np.rad2deg(np.arctan(m))
        # beta = 90 - alpha
        beta = np.rad2deg(np.arctan(1/m))           # angle from y-axis

    return [beta, dist]

def process_raw_gcode(gcode_df):
    # make line segments from raw gcode points
    # determine angle of all line segments
    # determine length of all line segments
    # turn into line segments
    # TODO: might make more sense to determine angle and length after points have been made into segments
    # TODO: delete travel moves

    segments_df = pd.DataFrame(columns=["p0","p1", "angle_d", "length"])

    # segment_df["index"] = 0
    # segments_df["p0"] = []
    # segments_df["p1"] = []
    segments_df["angle_d"] = 0
    segments_df["length"] = 0

    row = []
    index = 0
    
    cut_depth = min(gcode_df.z)

    for i in range(1, len(gcode_df)):
        print("pt1 = " + str([gcode_df.x[i - 1], gcode_df.y[i - 1]]))
        print("pt2 = " + str([gcode_df.x[i], gcode_df.y[i]]))
        [angle,length] = get_line_info(
            [gcode_df.x[i-1], gcode_df.y[i-1]], [gcode_df.x[i], gcode_df.y[i]]
        )

        # don't collect travel moves
        if (gcode_df.z[i] > cut_depth):
            continue

        point0 = [gcode_df.x[i-1], gcode_df.y[i-1], gcode_df.z[i-1]]
        point1 = [gcode_df.x[i], gcode_df.y[i], gcode_df.z[i]]

        # fill out temp list
        row.append({
            "p0": point0,
            "p1": point1,
            "angle_d": angle,
            "length": length
        })

        index = index + 1

    segments_df = pd.concat([segments_df, pd.DataFrame(row)], ignore_index=True)

    return segments_df

def convert_angles_to_radians(angles_deg):
    return np.radians(angles_deg)

def group_segments(segments_df):
    angle_range = 90     # this range is 2x the range of one side of y-axis

    # start with angle 0
    group_angle = 0

    k = 1           # can they all fit in one bucket?
    buckets_valid = 0

    while (not buckets_valid):
        kmeans_df = segments_df
        kmeans_df['x'] = np.cos(convert_angles_to_radians(kmeans_df.angle_d))
        kmeans_df['y'] = np.sin(convert_angles_to_radians(kmeans_df.angle_d))

        kmeans = KMeans(n_clusters=k, random_state=0).fit(kmeans_df[['x', 'y']])
        kmeans_df['cluster_label'] = kmeans.labels_

        buckets_valid = 1

        for cluster_label in np.unique(kmeans_df['cluster_label']):
            # Extract the angles in degrees for the current cluster
            cluster_angles = kmeans_df[kmeans_df['cluster_label'] == cluster_label]['angle_d']

            # Compute the min and max angles within the cluster
            min_angle = cluster_angles.min()
            max_angle = cluster_angles.max()

            # Handle wraparound at 360 degrees by considering both directions
            angular_range = min((max_angle - min_angle), (min_angle + 360 - max_angle))

            # Check if the angular range exceeds the boundary
            if angular_range > angle_range:
                k = k + 1
                buckets_valid = 0
                break

    cluster_centers = kmeans.cluster_centers_
    cluster_centers_deg = np.degrees(np.arctan2(cluster_centers[:, 1], cluster_centers[:, 0]))
    cluster_centers_deg = np.where(cluster_centers_deg > 180, cluster_centers_deg - 360, cluster_centers_deg)   # (-180,180)
    kmeans_df['cluster_angle'] = cluster_centers_deg

    # for label in kmeans_df['cluster_label']:
        

    # Reorganize dataframe
    group_df = pd.DataFrame(columns=['p0', 'p1', 'cluster_angle_d'])
    row = []
    for cluster_label in np.unique(kmeans_df['cluster_label']):
        point0 = kmeans_df[kmeans_df['cluster_label'] == cluster_label]['p0']
        point1 = kmeans_df[kmeans_df['cluster_label'] == cluster_label]['p1']
        cluster_angles = kmeans_df[kmeans_df['cluster_label'] == cluster_label]['angle_d']

        row.append({
            "p0": point0,
            "p1": point1,
            "cluster_angle": cluster_angles
        })
    group_df = pd.concat([group_df, pd.DataFrame(row)], ignore_index=True)

    return group_df

# Plotting
def plot_segments(gcode_df, filtered_segments):
    plt.figure(figsize=(10, 6))

    for segment in filtered_segments:
        segment_points = gcode_df.iloc[segment]
        x = segment_points["x"]
        y = segment_points["y"]

        plt.scatter(x, y, label="Segment Points")

        # Optionally plot lines connecting points in the segment
        plt.plot(x, y, linestyle="-", marker="o", label="Segment Path")

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Filtered Segments")
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_segment_by_group(grouped_segs_df):
    plt.figure(figsize=(10, 6))

    cmap = plt.get_cmap('tab10', len(grouped_segs_df))  # 'tab10' gives up to 10 distinct colors
    
    for idx in range(len(grouped_segs_df)):
        points0 = grouped_segs_df.p0[idx]
        points1 = grouped_segs_df.p1[idx]
        c = cmap(idx)

        for i in range(len(points0)):
            x = [points0[0], points1[0]]
            y = [points0[1], points1[1]]
            if i == 0:
                plt.plot(x, y, color=c, label=f'Cluster {idx}')
            else:
                plt.plot(x, y, color=c)


    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Plot of Segments by Angle")
    plt.grid(True)
    plt.legend()
    plt.show()


def main():
    # my_df = read_gcode_csv("dev/gCode/basePlate_test.csv")
    my_df = read_gcode_csv("dev/gCode/cal.csv")
    # my_df = read_gcode("dev/gCode/basePlate_test.nc")
    plt.scatter(my_df.x, my_df.y)
    plt.grid()
    plt.title("Plotting whole GCODE Path (just the points)")
    plt.show()

    gcode_df_processed = process_raw_gcode(my_df)
    grouped_segments_df = group_segments(gcode_df_processed)
    plot_segment_by_group(grouped_segments_df)


if __name__ == "__main__":
    main()
