import numpy as np
import cv2
import glob


#Calibration board
cb_width = 9 #Number of intersections
cb_height = 6
cb_square_size = 25.0 #[mm]

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cb_3D_points = np.zeros((cb_width * cb_height, 3), np.float32)
cb_3D_points[:,:2] = np.mgrid[0:cb_width, 0:cb_height].T.reshape(-1,2) * cb_square_size

list_cb_3d_points = [] #3D points in real world space
list_cb_2d_img_points = [] #2D points in image plane

list_images = glob.glob('calibration_images/*.jpg')

for frame_name in list_images:
    img = cv2.imread(frame_name)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (cb_width,cb_height), None)

    if ret:
        list_cb_3d_points.append(cb_3D_points)

        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        list_cb_2d_img_points.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (cb_width, cb_height), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(list_cb_3d_points, list_cb_2d_img_points, gray.shape[::-1], None, None)

print('Calibration matrix:')
print(mtx)
print('Distortion')
print(dist)

with open('matrix_distortion_logitech_c920.npy', 'wb') as f:
    np.save(f, mtx)
    np.save(f, dist)
