"""
1. Description of Aruco library functions: https://docs.opencv.org/4.5.5/d9/d6a/group__aruco.html#ga84dd2e88f3e8c3255eb78e0f79571bd1
2. Transition from CS marker to CS camera: https://stackoverflow.com/questions/46363618/aruco-markers-with-opencv-get-the-3d-corner-coordinates
3. About PatternPos and CornerRefineMethod: https://docs.opencv.org/3.4/d9/d6a/group__aruco.html#gafce26321f39d331bc12032a72b90eda6
"""

from math import radians, degrees
import numpy as np
import cv2
import cv2.aruco as aruco
import paho.mqtt.client as mqtt
from CoordinatesTransformation import getTransformedCoords


# MQTT setup
broker = "172.31.1.207"
port = 1883
username = 'broker_name'
password = 'broker_password'


def initializeMqttClient():
    client = mqtt.Client("RPi4")
    client.username_pw_set(username, password)
    client.connect(broker, port)
    return client

def publishCoordinates(pub_topic, m_id):
    coords = dict_marker_center_coords.get(m_id)
    if coords is None:
        client.publish(pub_topic, "Marker coordinates with given ID not found")
    else:
        client.publish(pub_topic, str(coords.tolist()))

def on_message(client, userdata, msg):
    global marker_size
    global marker_id
    message = msg.payload.decode()
    message_topic = msg.topic
    if message_topic == 'data/coordsFromCamera':
        global sendCoordsFormCameraFlag
        sendCoordsFormCameraFlag = True
        message = message[1:-1]
        message = message.split(", ")
        message = [float(i) for i in message]
        marker_id = int(message[0])
        marker_size = message[1]
    if message_topic == 'data/transformCoords':
        global sendTransformCoordsFlag
        global flangeCoords
        sendTransformCoordsFlag = True
        message = message[1:-1]
        message = message.split(", ")
        message = [float(i) for i in message]
        flangeCoords = message[0:6]
        marker_id = int(message[6])
        marker_size = message[7]

def calculateCenterCoordinates(rv, tv):
    """
    Arguments:
    rv -- rotation coordinates of the marker center relative to the camera's CS
    tv -- translation coordinates of the marker center in the camera's CS

    Returns:
    coords_center -- marker center coordinates
    """
    tv = tv.reshape(3, 1)
    Rmat, _ = cv2.Rodrigues(rv)  #Rotation matrix, Jacobian matrix
    angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(Rmat)  #angles -- orientation of the marker center relative to the SC camera
    coords_center = np.round(np.array( [tv[0][0], tv[1][0], tv[2][0], radians(angles[2]), radians(angles[1]), radians(angles[0])] ), 2)
    return coords_center

client = initializeMqttClient()
client.on_message = on_message
client.subscribe("data/coordsFromCamera", qos=2)
client.subscribe("data/transformCoords", qos=2)

with open('matrix_distortion_logitech_c920.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

cap = cv2.VideoCapture(0)
width = 1920
height = 1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, 30)

marker_id = 0
marker_size = 30 #[мм]
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters_create()
arucoParams.cornerRefinementMethod = aruco.CORNER_REFINE_NONE

flangeCoords = []
coords_from_cam = []
sendTransformCoordsFlag = False
sendCoordsFormCameraFlag = False
dict_marker_center_coords = {}
np.set_printoptions(suppress=True) #Suppress e in float numbers
count = 0
dx_pixel = -50
dy_pixel = -5


while True:
    count += 1
    client.loop_start()
    _, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray_frame, aruco_dict, parameters=arucoParams, cameraMatrix=camera_matrix, distCoeff=camera_distortion) #corners -- coordinates of marker corners in pixels
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

    # Display the camera's coordinate system (red - X, green - Y)
    cv2.line(frame, (int(width/2 + dx_pixel), int(height/2 + dy_pixel)), (int(width/2+300), int(height/2) + dy_pixel), (0, 0, 255), 1) #X
    cv2.line(frame, (int(width/2 + dx_pixel), int(height/2 + dy_pixel)), (int(width/2 + dx_pixel), int(height/2+300)), (0, 255, 0), 1) #Y

    # If the camera finds markers
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners) #Display a frame with a square in the corner on the image (circle the marker)
        for i, id in enumerate(ids):
            marker_center_coords = calculateCenterCoordinates(rvecs[i], tvecs[i])
            dict_marker_center_coords[id[0]] = marker_center_coords

            # Display the axes of the i-th marker
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvecs[i], tvecs[i], 30)

            # Display id in the upper left corner
            # corners[1][2][3][4]: 1-[[[for a specific id]]]; 2-[[ ]]; 3-coordinates of 1 corner; 4-coordinate x/y
            cv2.putText(frame, str(id[0]), (int(corners[i][0][0][0]) - 25, int(corners[i][0][0][1])), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)

            # Draw coordinates of the marker center and point
            tvec_str = "Center:  x:%7.1f   y:%7.1f   z:%7.1f   a:%7.1f   b:%7.1f   c:%7.1f" % (marker_center_coords[0], marker_center_coords[1], marker_center_coords[2],
                                                                                               marker_center_coords[3], marker_center_coords[4], marker_center_coords[5])
            # if id[0] == 39:
            #     cv2.putText(frame, tvec_str, (5, 20), cv2.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 255), 2, cv2.LINE_AA)
            #     print(marker_center_coords[0], marker_center_coords[1], marker_center_coords[2], degrees(marker_center_coords[3]), degrees(marker_center_coords[4]), degrees(marker_center_coords[5]))
    else:
        dict_marker_center_coords.clear()

    if sendTransformCoordsFlag:
        coords_from_cam = dict_marker_center_coords.get(marker_id)
        if coords_from_cam is None:
            client.publish('state/transformCoords', "Marker coordinates with given ID not found")
        else:
            new_coords = getTransformedCoords(flangeCoords, coords_from_cam)
            client.publish('state/transformCoords', str(new_coords.tolist()))
        sendTransformCoordsFlag = False

    if sendCoordsFormCameraFlag:
        publishCoordinates("state/coordsFromCamera", marker_id)
        sendCoordsFormCameraFlag = False

    flag = False
    cv2.imshow('Frame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
