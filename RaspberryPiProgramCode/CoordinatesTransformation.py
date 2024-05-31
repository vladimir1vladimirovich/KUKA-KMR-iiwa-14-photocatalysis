import numpy as np
from numpy.linalg import multi_dot
from scipy.spatial.transform import Rotation
from math import degrees, radians, pi


np.set_printoptions(suppress=True)

deltaCameraX = -78.2
deltaCameraY = 1.0
deltaCameraZ = -22.0

def getTransformedCoords(F_cords, C_cords):
    x_flange = F_cords[0]
    y_flange = F_cords[1]
    z_flange = F_cords[2]
    a_flange = F_cords[3]  # around Z
    b_flange = F_cords[4]  # around Y
    c_flange = F_cords[5]  # around X
    x_marker_in_cam = C_cords[0]
    y_marker_in_cam = C_cords[1]
    z_marker_in_cam = C_cords[2]
    a_marker_in_cam = C_cords[3]
    b_marker_in_cam = C_cords[4]
    c_marker_in_cam = C_cords[5]
    point_in_marker = np.array([0.0, 0.0, 0.0, 1]).reshape(4, 1)

    # Transformation 1
    R_K_F = Rotation.from_euler('xyz', [c_flange, b_flange, a_flange]).as_matrix()
    d_K_F = np.array([x_flange, y_flange, z_flange])
    HTM_K_F = np.zeros((4,4)) #Homogeneous Transformation Matrix
    HTM_K_F[0:3,0:3] = R_K_F
    HTM_K_F[0:3,3] = d_K_F
    HTM_K_F[3,3] = 1

    ## Transformation 2
    R_F_C = Rotation.from_euler('z', [-90], degrees=True).as_matrix()[0]
    d_F_C = np.array([deltaCameraX, deltaCameraY, deltaCameraZ])
    HTM_F_C = np.zeros((4,4))
    HTM_F_C[0:3,0:3] = R_F_C
    HTM_F_C[0:3,3] = d_F_C
    HTM_F_C[3,3] = 1

    # Transformation 3
    R_C_M = Rotation.from_euler('xyz', [c_marker_in_cam, b_marker_in_cam, a_marker_in_cam]).as_matrix()
    d_C_M = np.array([x_marker_in_cam, y_marker_in_cam, z_marker_in_cam])
    HTM_C_M = np.zeros((4,4))
    HTM_C_M[0:3,0:3] = R_C_M
    HTM_C_M[0:3,3] = d_C_M
    HTM_C_M[3,3] = 1

    # Transformation 4
    HTM = multi_dot([HTM_K_F, HTM_F_C, HTM_C_M])
    point_in_kuka = HTM @ point_in_marker

    R1 = np.array([[0, -1, 0],
                   [-1, 0, 0],
                   [0, 0, -1]])
    R = HTM[0:3, 0:3]
    R_total = R @ R1
    R_total = Rotation.from_matrix(R_total)
    angles = R_total.as_euler('xyz')

    return np.array([point_in_kuka[0][0], point_in_kuka[1][0], point_in_kuka[2][0], angles[2], angles[1], angles[0]])