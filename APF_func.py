import numpy as np

def shortest_vector(point, point_cloud):
    #find the shortest distance between a point and a boundary

    # Calculate the distances from the single point to all points in the point cloud
    distances = np.linalg.norm(point_cloud - point, axis=1)

    # Find the shortest distance and the corresponding point
    closest_point = point_cloud[np.argmin(distances)]
    shortest_vector = closest_point - point

    return shortest_vector, closest_point