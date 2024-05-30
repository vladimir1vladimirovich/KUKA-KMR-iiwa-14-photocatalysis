import cv2
import time


cv2.namedWindow('Camera image')
cv2.moveWindow('Camera image', 0, 0)


# Setup camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)


prev_frame_time = time.time()

calibration_image_count = 0
frame_count = 0


while True:
    _, frame = cap.read()

    # Делаем калибровочное фото каждые 15 фреймов
    frame_count += 1
    if frame_count == 15:
        cv2.imwrite("calibration_images/" + 'calibration_image_' + str(calibration_image_count) + '.jpg', frame)
        calibration_image_count += 1
        frame_count = 0

    # Calculate the FPS and display on frame
    new_frame_time = time.time()
    fps = 1/(new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time

    cv2.putText(frame, 'FPS ' + str(int(fps)), (10, 30), cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow('Camera image', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
