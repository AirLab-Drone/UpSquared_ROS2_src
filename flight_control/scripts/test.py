import cv2 

cap = cv2.VideoCapture(0)
count = 0
start_time = cv2.getTickCount()

while True:

    ret, frame = cap.read()
    count += 1
    if(count%30 == 0):
        end_time = cv2.getTickCount()
        print('FPS: ', 30/((end_time - start_time)/cv2.getTickFrequency()))
        start_time = cv2.getTickCount()
    cv2.waitKey(1)