import numpy as np

def shortest_dist(point, point_cloud):
    #find the shortest distance between a point and a boundary

    # Calculate the distances from the single point to all points in the point cloud
    distances = np.linalg.norm(point_cloud - single_point, axis=1)

    # Find the shortest distance and the corresponding point
    shortest_distance = np.min(distances)
    closest_point = point_cloud[np.argmin(distances)]

    return shortest_distance, closest_point