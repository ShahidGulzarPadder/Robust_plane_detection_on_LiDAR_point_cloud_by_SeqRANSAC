# Robust_plane_detection_on_LiDAR_point_cloud_by_SeqRANSAC
Sequential RANSAC is the iterative modification of RANSAC: it finds a plane by RANSAC, then the plane points are removed from the dataset, and RANSAC method is run again on the rest of the points. The plane detection and removal can be iterated many times. In this assignment, the three most dominant plane should be found. 
