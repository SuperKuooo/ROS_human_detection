import cv2

print(cv2.__version__)
video = cv2.VideoCapture(1)

while True:

    ret, frame = video.read()

    cv2.imshow('Object detector', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break
