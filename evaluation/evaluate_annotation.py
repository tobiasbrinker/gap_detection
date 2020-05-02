"""
This script will read out an annotation file and create a
annotation_evaluation.txt listing the labels, the number of points and volume
of each gap as well as the center point for each gap.

Notes
-----
This script requires a .pcd file of the annotated point cloud where non-gap
points are labbeled 0. The points of a gap should be labelled under the same
number with the first gap starting at 1. Furthermore the depth axis has to be
specified.

The volume is estimated using a convex hull and the Qhull library.

Usage
-----
python evaluate_annotation.py <annotation-filename> <depth-axis>

Example call:
python evaluate_annotation.py example-annotation/labelled.pcd x

"""

import sys
import numpy as np
import collections
from scipy.spatial import ConvexHull
import imp
helper_module = imp.load_source('helpers', '../src/helpers.py')


if(len(sys.argv) != 3):
    print("Please enter the annotation file as an argument first and " +
          " then the depth axis!")
    print("python evaluate_annotation.py [annotation_file] [depth_axis]")
else:
    # ----- ARGUMENT PARSER -----
    file_path = sys.argv[1]

    if(sys.argv[2] == "x"):
        depth_axis = 0
    elif(sys.argv[2] == "y"):
        depth_axis = 1
    elif(sys.argv[2] == "z"):
        depth_axis = 2
    else:
        print("Not a valid depth axis argument. Valid inputs: x,y and z.")
        sys.exit()

    try:
        annotation = open(file_path)
    except (IOError, OSError):
        print("Could not open/read file: " + file_path)
        sys.exit()

    # ----- DATA PREPROCESSING -----
    pcd_data = annotation.read()
    point_data = pcd_data.split("ascii", 1)[1]
    rows = point_data.split("\n")
    rows.pop(0)

    row_data = []
    for row in rows:
        r = row.split(" ")

        if(len(r) == 5):
            row_data.append(map(float, r))

    clustered_points = []
    surface_points = []
    label_list = []
    for row in row_data:
        if(row[3] == 0.0):
            surface_points.append(row)
        if(row[3] != 0.0):  # 0.0 is the label for not annotated
            clustered_points.append(row)
            label_list.append(row[3])

    # ----- NUMBER OF POINTS -----
    label_counter = collections.Counter(label_list)
    number_of_points = [gap[1] for gap in label_counter.items()]

    # ----- VOLUME -----
    clusters = []
    clustered_points = np.array(clustered_points)
    for i in set(label_list):
        cluster = clustered_points[clustered_points[:, 3] == i]
        cluster = cluster[:, [0, 1, 2]]

        # To construct a convex hull a minimum of 4 points is needed
        num_of_points, dim = cluster.shape
        if(num_of_points >= 4):
            clusters.append(cluster)

    # ----- VOLUME CORRECTION -----
    volume_corrected_clusters = []
    surface_points = np.array(surface_points)
    surface_height = np.median(surface_points[:, depth_axis])

    for cluster in clusters:
        hull = ConvexHull(cluster, qhull_options="QJ")

        # Map from vertex to point in cluster
        vertices = []
        for vertex in hull.vertices:
            x, y, z = cluster[vertex]
            vertices.append([x, y, z])

        gap = cluster.tolist()
        for vertex in vertices:
            # For each vertex, add a new point to the gap with the height
            # of the surface and the other axes corresponding to the vertex
            if(depth_axis == 0):
                volume_point = [surface_height, vertex[1], vertex[2]]
            elif(depth_axis == 1):
                volume_point = [vertex[0], surface_height, vertex[2]]
            elif(depth_axis == 2):
                volume_point = [vertex[0], vertex[1], surface_height]

            gap.append(volume_point)

        volume_corrected_clusters.append(np.array(gap))

    # ---- CALCULATING CONVEX HULLS OF GAPS AND THEIR CENTER -----
    convex_hulls_and_info = helper_module.calculate_convex_hulls_and_centers(
        volume_corrected_clusters)

    convex_hulls_and_info = np.array(convex_hulls_and_info)
    volumes = convex_hulls_and_info[:, 3]
    centers = convex_hulls_and_info[:, 0]
    evaluation_info = zip(number_of_points, volumes, centers)

    # ---- WRITE THE RESULTS TO A FILE -----
    try:
        file = open("annotation_evaluation.txt", "w")
    except (IOError, OSError):
        print("Could not write file: annotation_evaluation.txt")
        sys.exit()

    file.write("Annotation Evaluation of " + file_path + "\n\n")
    file.write("Gap\t\tNumber of points\t\tVolume (in cm^3)\t" +
               "\tCenter Point\n")
    for gap_number, gap in enumerate(evaluation_info):
        number_of_points, volume, center = gap
        file.write(str(int(gap_number)+1) + "\t\t" +
                   str(number_of_points) + "\t\t\t\t" +
                   str(volume*1000000) + "\t\t\t" +
                   str(center) + "\n")

    file.close()
    print("Saved evaluation report of annotation to annotation_evaluation.txt")
